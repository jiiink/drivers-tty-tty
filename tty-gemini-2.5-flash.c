#include <assert.h>
#include <minix/drivers.h>
#include <minix/driver.h>
#include <termios.h>
#include <sys/kbdio.h>
#include <sys/ttycom.h>
#include <sys/ttydefaults.h>
#include <sys/fcntl.h>
#include <signal.h>
#include <minix/keymap.h>
#include "tty.h"
#include <sys/time.h>
#include <sys/select.h>

unsigned long rs_irq_set = 0;

#define tty_addr(line)	(&tty_table[line])
#define isconsole(tp)	((tp) < tty_addr(NR_CONS))
#define FIRST_TTY	tty_addr(0)
#define END_TTY		tty_addr(sizeof(tty_table) / sizeof(tty_table[0]))
#define tty_active(tp)	((tp)->tty_devread != NULL)

#if NR_RS_LINES == 0
#define rs_init(tp)	((void) 0)
#endif

struct kmessages kmess;

static void tty_timed_out(int arg);
static void settimer(tty_t *tty_ptr, int enable);
static void in_transfer(tty_t *tp);
static int tty_echo(tty_t *tp, int ch);
static void rawecho(tty_t *tp, int ch);
static int back_over(tty_t *tp);
static void reprint(tty_t *tp);
static void dev_ioctl(tty_t *tp);
static void setattr(tty_t *tp);
static void tty_icancel(tty_t *tp);
static void tty_init(void);
static void do_new_kmess(void);
static void set_console_line(char term[CONS_ARG]);
static void set_kernel_color(char color[CONS_ARG]);
static void set_color(tty_t *tp, int color);
static void reset_color(tty_t *tp);

static int do_open(devminor_t minor, int access, endpoint_t user_endpt);
static int do_close(devminor_t minor);
static ssize_t do_read(devminor_t minor, u64_t position, endpoint_t endpt,
	cp_grant_id_t grant, size_t size, int flags, cdev_id_t id);
static ssize_t do_write(devminor_t minor, u64_t position, endpoint_t endpt,
	cp_grant_id_t grant, size_t size, int flags, cdev_id_t id);
static int do_ioctl(devminor_t minor, unsigned long request, endpoint_t endpt,
	cp_grant_id_t grant, int flags, endpoint_t user_endpt, cdev_id_t id);
static int do_cancel(devminor_t minor, endpoint_t endpt, cdev_id_t id);
static int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt);

static struct chardriver tty_tab = {
	.cdr_open	= do_open,
	.cdr_close	= do_close,
	.cdr_read	= do_read,
	.cdr_write	= do_write,
	.cdr_ioctl	= do_ioctl,
	.cdr_cancel	= do_cancel,
	.cdr_select	= do_select
};

static struct termios termios_defaults = {
  .c_iflag = TTYDEF_IFLAG,
  .c_oflag = TTYDEF_OFLAG,
  .c_cflag = TTYDEF_CFLAG,
  .c_lflag = TTYDEF_LFLAG,
  .c_ispeed = TTYDEF_SPEED,
  .c_ospeed = TTYDEF_SPEED,
  .c_cc = {
	[VEOF] = CEOF,
	[VEOL] = CEOL,
	[VERASE] = CERASE,
	[VINTR] = CINTR,
	[VKILL] = CKILL,
	[VMIN] = CMIN,
	[VQUIT] = CQUIT,
	[VTIME] = CTIME,
	[VSUSP] = CSUSP,
	[VSTART] = CSTART,
	[VSTOP] = CSTOP,
	[VREPRINT] = CREPRINT,
	[VLNEXT] = CLNEXT,
	[VDISCARD] = CDISCARD,
	[VSTATUS] = CSTATUS
  }
};
static struct winsize winsize_defaults;

tty_t tty_table[NR_CONS+NR_RS_LINES];
int ccurrent;
struct machine machine;
u32_t system_hz;
u32_t consoleline = CONS_MINOR;
u32_t kernel_msg_color = 0;

static const char lined[TTLINEDNAMELEN] = "termios";

static void sef_local_startup(void);
static int sef_cb_init_fresh(int type, sef_init_info_t *info);
static void sef_cb_signal_handler(int signo);

static void process_tty_events(void)
{
    tty_t *tp;
    for (tp = FIRST_TTY; tp < END_TTY; tp++) {
        if (tp == NULL) {
            continue;
        }
        if (tp->tty_events) {
            handle_events(tp);
        }
    }
}

static void handle_kernel_notification(message *tty_mess, int ipc_status)
{
	if (!is_ipc_notify(ipc_status)) {
		return;
	}

	if (tty_mess == NULL) {
		return;
	}

	int should_expire_timers = 0;

	switch (_ENDPOINT_P(tty_mess->m_source)) {
	case CLOCK:
		should_expire_timers = 1;
		break;
	case HARDWARE:
#if NR_RS_LINES > 0
		if ((tty_mess->m_notify.interrupts & rs_irq_set) != 0) {
			rs_interrupt(tty_mess);
		}
#endif
		should_expire_timers = 1;
		break;
	default:
		break;
	}

	if (should_expire_timers) {
		expire_timers(tty_mess->m_notify.timestamp);
	}
}

static int handle_special_message(message *tty_mess, int ipc_status, int line)
{
	switch (tty_mess->m_type) {
	case TTY_FKEY_CONTROL:
		do_fkey_ctl(tty_mess);
		return 1;
	case TTY_INPUT_UP:
	case TTY_INPUT_EVENT:
		do_input(tty_mess);
		return 1;
	default:
		break;
	}
	
	if (!IS_CDEV_RQ(tty_mess->m_type)) {
		chardriver_process(&tty_tab, tty_mess, ipc_status);
		return 1;
	} else if (line == VIDEO_MINOR) {
		do_video(tty_mess, ipc_status);
		return 1;
	}
	
	return 0;
}

int main(void)
{
	message tty_mess = {0};
	int ipc_status = 0;
	int r = 0;

	sef_local_startup();
	
	while (TRUE) {
		process_tty_events();

		r = driver_receive(ANY, &tty_mess, &ipc_status);
		if (r != 0) {
			panic("driver_receive failed with: %d", r);
		}

		if (is_ipc_notify(ipc_status)) {
			handle_kernel_notification(&tty_mess, ipc_status);
			continue;
		}
		
		int line = 0;
		if (OK != chardriver_get_minor(&tty_mess, &line)) {
			continue;
		}
			
		if (handle_special_message(&tty_mess, ipc_status, line)) {
			continue;
		}

		chardriver_process(&tty_tab, &tty_mess, ipc_status);
	}
}

static void set_color(tty_t *tp, int color)
{
	// Allocate a buffer large enough for common SGR sequences like "\033[1;107m" plus null terminator.
	// \033 (1 byte) + [1; (3 bytes) + color (max 3 digits for 100-107) + m (1 byte) + \0 (1 byte)
	// Total: 1 + 3 + 3 + 1 + 1 = 9 bytes.
	char buf[9];
	int snprintf_res; // Store the return value of snprintf

	buf[0] = '\033'; // Start with the ASCII Escape character

	// Format the rest of the sequence into the buffer, starting at buf[1].
	// sizeof(buf) - 1 ensures space for the null terminator.
	// snprintf returns the number of characters that would have been written,
	// excluding the null byte, if the buffer was large enough.
	// It guarantees null termination if the buffer size argument is greater than 0.
	snprintf_res = snprintf(&buf[1], sizeof(buf) - 1, "[1;%dm", color);

	// Determine the actual number of bytes to write.
	// strlen(buf) correctly measures the length of the formatted string
	// (including buf[0] and any characters written by snprintf), up to the null terminator.
	// This ensures we do not send the null terminator itself to the terminal,
	// and also correctly handles truncation if 'color' was too large for the buffer,
	// as snprintf would null-terminate the truncated string.
	size_t total_len_to_write = strlen(buf);

	// Perform the write operation.
	// The original code passed sizeof(buf), which incorrectly included the null terminator.
	// Using total_len_to_write (from strlen) ensures only the escape sequence is sent.
	// The cast (cp_grant_id_t) buf is kept as in the original code,
	// assuming this specific system's do_write function expects a pointer despite the type name.
	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) buf, total_len_to_write,
		CDEV_NONBLOCK, 0);
}

static void reset_color(tty_t *tp)
{
    const int sgr_color_reset_code = 39;

    char cmd_buffer[8];

    int formatted_len = snprintf(cmd_buffer, sizeof(cmd_buffer), "\033[0;%dm", sgr_color_reset_code);

    if (formatted_len < 0 || (size_t)formatted_len >= sizeof(cmd_buffer)) {
        return;
    }

    int write_result = do_write(tp->tty_minor, 0, KERNEL, (const cp_grant_id_t)cmd_buffer, formatted_len,
                                CDEV_NONBLOCK, 0);

    if (write_result < 0) {
        return;
    }
}

tty_t *line2tty(devminor_t line)
{
    // Handle special minor numbers that redirect
    if (line == CONS_MINOR || line == LOG_MINOR) {
        line = consoleline;
    }

    // Handle VIDEO_MINOR explicitly; it never corresponds to an active TTY.
    if (line == VIDEO_MINOR) {
        return NULL;
    }

    int tty_idx = -1; // Initialize to an invalid index

    // Determine the type of TTY and calculate its global index.
    // Ensure 'line' is within the valid range for each type.
    if (line >= CONS_MINOR && line < (CONS_MINOR + NR_CONS)) {
        tty_idx = line - CONS_MINOR;
    } else if (line >= RS232_MINOR && line < (RS232_MINOR + NR_RS_LINES)) {
        // RS232 TTYs are indexed starting after all console TTYs.
        tty_idx = (line - RS232_MINOR) + NR_CONS;
    }

    // Now, validate the calculated index and retrieve the TTY pointer.
    tty_t* tp = NULL; // Default to NULL if no TTY found or invalid index.

    // Define the total expected number of TTYs for bounds checking.
    // This constant helps prevent potential out-of-bounds access.
    const int total_tty_count = NR_CONS + NR_RS_LINES;

    if (tty_idx >= 0 && tty_idx < total_tty_count) {
        tp = tty_addr(tty_idx);
        // After retrieving, check if the TTY is active. If not, treat as non-existent.
        if (tp != NULL && !tty_active(tp)) {
            tp = NULL;
        }
    }

    return tp;
}

static void sef_local_startup(void)
{
    int ret;

    ret = sef_setcb_init_fresh(sef_cb_init_fresh);
    if (ret != 0) {
        return;
    }

    ret = sef_setcb_init_restart(SEF_CB_INIT_RESTART_STATEFUL);
    if (ret != 0) {
        return;
    }

    ret = sef_setcb_signal_handler(sef_cb_signal_handler);
    if (ret != 0) {
        return;
    }

    ret = sef_startup();
    if (ret != 0) {
        return;
    }
}

static int sef_cb_init_fresh(int UNUSED(type), sef_init_info_t *UNUSED(info))
{
	int status;
	char val[CONS_ARG];

	if ((status = sys_getmachine(&machine)) != OK) {
		panic("Couldn't obtain kernel environment: %d", status);
	}

	if (env_get_param("console", val, sizeof(val)) == OK) {
		set_console_line(val);
	}

	if (env_get_param("kernelclr", val, sizeof(val)) == OK) {
		set_kernel_color(val);
	}

	tty_init();
	kb_init_once();
	sys_diagctl_register();

	return OK;
}

#include <string.h> // For strncmp, strlen, strlcpy
#include <assert.h> // For assert
#include <stddef.h> // For size_t

// Define MIN macro for clarity and to avoid complex ternary expressions.
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

// Helper function to safely generate console names following the "ttyXN" pattern.
// Examples: "ttyc0", "tty00" as base templates.
// The `base_pattern_template` must be 5 characters long, with the last character
// being a digit (e.g., '0'), which will be modified by `index`.
// Requires `buffer` of size at least 6 for "ttyXN\0".
// Returns 0 on success, -1 on error (e.g., buffer too small, invalid template, or index out of range).
static int generate_tty_name(char *buffer, size_t buffer_size, const char *base_pattern_template, int index)
{
    const size_t PATTERN_LENGTH = 5;      // e.g., strlen("ttyc0")
    const size_t PATTERN_FULL_LENGTH = 6; // e.g., sizeof("ttyc0") for null-termination

    // Ensure the provided buffer is large enough to hold the generated string.
    if (buffer_size < PATTERN_FULL_LENGTH) {
        return -1; // Buffer too small.
    }

    // Validate the base_pattern_template structure.
    if (strlen(base_pattern_template) != PATTERN_LENGTH) {
        return -1; // Invalid template length (expected "ttyXN").
    }
    if (base_pattern_template[PATTERN_LENGTH - 1] < '0' || base_pattern_template[PATTERN_LENGTH - 1] > '9') {
        return -1; // The last character of the template must be a digit.
    }

    // Ensure the index is a single digit, consistent with character arithmetic.
    if (index < 0 || index > 9) {
        return -1; // Index out of expected range for single digit.
    }

    // Copy the base pattern (e.g., "ttyc0") into the buffer.
    // strlcpy ensures null-termination and prevents buffer overflows.
    if (strlcpy(buffer, base_pattern_template, buffer_size) >= buffer_size) {
        // This case should ideally not be reached if buffer_size and template are valid.
        return -1;
    }

    // Modify the last character with the given index.
    // Example: '0' + 1 results in '1'.
    buffer[PATTERN_LENGTH - 1] = base_pattern_template[PATTERN_LENGTH - 1] + index;

    return 0; // Success.
}


static void set_console_line(char term[CONS_ARG])
{
    // Calculate the maximum number of characters to compare from the 'term' buffer.
    // This value mirrors the `CONS_ARG - 1` used in the original code,
    // which effectively limits the comparison length based on the input buffer's capacity.
    // Ensure this value doesn't underflow if CONS_ARG is 0 (though CONS_ARG should be > 0).
    const size_t term_compare_limit = (CONS_ARG > 0) ? (CONS_ARG - 1) : 0;

    // --- First, check for "console" ---
    const char *console_str = "console";
    // The original code compared up to `CONS_ARG - 1` characters.
    // This allows for partial matches if `CONS_ARG - 1` is less than `strlen("console")`.
    if (!strncmp(term, console_str, term_compare_limit)) {
        consoleline = CONS_MINOR + 0;
        return;
    }

    // --- Next, iterate through "ttyc1" to "ttyc(NR_CONS-1)" ---
    // The loop starts from 1 as per the original logic.
    for (int i = 1; i < NR_CONS; i++) {
        char cons_name[6]; // Buffer sufficient for "ttycX\0" (e.g., "ttyc1\0")

        // Generate the console name using the helper function.
        if (generate_tty_name(cons_name, sizeof(cons_name), "ttyc0", i) == 0) {
            // `strlen(cons_name)` will be 5 for patterns like "ttyc1".
            const size_t cons_name_len = strlen(cons_name);

            // The original comparison length was `MIN(CONS_ARG - 1, sizeof(cons) - 1)`.
            // `sizeof(cons) - 1` is 5 for the `cons_name` buffer.
            const size_t comparison_len = MIN(term_compare_limit, cons_name_len);

            if (!strncmp(term, cons_name, comparison_len)) {
                consoleline = CONS_MINOR + i;
                return;
            }
        }
    }

    // --- Assert for serial line count compatibility ---
    // This assert clarifies a critical precondition: `NR_RS_LINES` must be single-digit
    // for the 'tty0X' naming scheme (where 'X' is a single digit) to work correctly.
    assert(NR_RS_LINES <= 9);

    // --- Finally, iterate through "tty00" to "tty0(NR_RS_LINES-1)" ---
    // The loop starts from 0 as per the original logic.
    for (int i = 0; i < NR_RS_LINES; i++) {
        char sercons_name[6]; // Buffer sufficient for "tty0X\0" (e.g., "tty00\0")

        // Generate the serial console name using the helper function.
        if (generate_tty_name(sercons_name, sizeof(sercons_name), "tty00", i) == 0) {
            // `strlen(sercons_name)` will be 5 for patterns like "tty00".
            const size_t sercons_name_len = strlen(sercons_name);

            // The original comparison length was `MIN(CONS_ARG - 1, sizeof(sercons) - 1)`.
            // `sizeof(sercons) - 1` is 5 for the `sercons_name` buffer.
            const size_t comparison_len = MIN(term_compare_limit, sercons_name_len);

            if (!strncmp(term, sercons_name, comparison_len)) {
                consoleline = RS232_MINOR + i;
                return;
            }
        }
    }
}

#include <stdlib.h>
#include <limits.h>
#include <errno.h>
#include <ctype.h>

static const int SGR_COLOR_START = 30;
static const int SGR_COLOR_END = 37;

static void set_kernel_color(const char *color_str)
{
    long val;
    char *endptr;

    errno = 0;

    val = strtol(color_str, &endptr, 10);

    if (endptr == color_str) {
        return;
    }

    if (errno == ERANGE || val > INT_MAX || val < INT_MIN) {
        return;
    }

    while (*endptr != '\0' && isspace((unsigned char)*endptr)) {
        endptr++;
    }
    if (*endptr != '\0') {
        return;
    }

    int def_color = (int)val;

    if (def_color >= 0 && def_color <= (SGR_COLOR_END - SGR_COLOR_START)) {
        kernel_msg_color = def_color + SGR_COLOR_START;
    }
}

static void copy_kernel_messages(char *kernel_buf_copy, struct kmessages *kmess_ptr, 
                                 int prev_next, int next, int bytes)
{
	if (kernel_buf_copy == NULL || kmess_ptr == NULL) {
		return;
	}

	if (prev_next < 0 || prev_next >= (int)_KMESS_BUF_SIZE) {
		return;
	}

	if (bytes <= 0) {
		return;
	}

	size_t buffer_size = (size_t)_KMESS_BUF_SIZE;
	size_t start_offset = (size_t)prev_next;
	size_t bytes_to_copy = (size_t)bytes;

	size_t copy_first_part = MIN(buffer_size - start_offset, bytes_to_copy);

	memcpy(kernel_buf_copy, &kmess_ptr->km_buf[start_offset], copy_first_part);

	if (copy_first_part < bytes_to_copy) {
		size_t remaining_bytes = bytes_to_copy - copy_first_part;
		memcpy(&kernel_buf_copy[copy_first_part], &kmess_ptr->km_buf[0], remaining_bytes);
	}

	(void)next; /* Explicitly mark 'next' as unused to suppress warnings */
}

static void do_new_kmess(void)
{
	const struct minix_kerninfo *kinfo = get_minix_kerninfo();
	if (kinfo == NULL) {
		panic("Kernel information structure is NULL.");
	}

	const struct kmessages *kmess_ptr = kinfo->kmessages;
	if (kmess_ptr == NULL) {
		panic("Kernel messages structure is NULL.");
	}

	char kernel_buf_copy[_KMESS_BUF_SIZE];
	static int prev_next_pos = 0;
	int current_next_pos = kmess_ptr->km_next;
	int bytes_to_copy = ((current_next_pos + _KMESS_BUF_SIZE) - prev_next_pos) % _KMESS_BUF_SIZE;
	
	if (bytes_to_copy == 0) {
		/* No new messages, prev_next_pos is already current_next_pos. */
		return;
	}

	copy_kernel_messages(kernel_buf_copy, kmess_ptr, prev_next_pos, current_next_pos, bytes_to_copy);

	tty_t *tp = line2tty(consoleline);
	if (tp == NULL) {
		panic("Don't know where to send kernel messages (consoleline invalid)");
	}
		
	int saved_tty_outleft = -1; /* Sentinel value to indicate no save/restore needed */

	if (tp->tty_outleft > 0) {
		saved_tty_outleft = tp->tty_outleft;
		tp->tty_outleft = 0;
	}

	if (kernel_msg_color != 0) {
		set_color(tp, kernel_msg_color);
	}
		
	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) kernel_buf_copy, 
	         bytes_to_copy, CDEV_NONBLOCK, 0);
	         
	if (kernel_msg_color != 0) {
		reset_color(tp);
	}
		
	if (saved_tty_outleft != -1) {
		tp->tty_outleft = saved_tty_outleft;
	}

	prev_next_pos = current_next_pos;
}

#include <signal.h>

static volatile sig_atomic_t sef_sigkmess_received = 0;
static volatile sig_atomic_t sef_sigterm_received = 0;

static void sef_cb_signal_handler(int signo)
{
	switch(signo) {
	case SIGKMESS:
		sef_sigkmess_received = 1;
		break;
	case SIGTERM:
		sef_sigterm_received = 1;
		break;
	default:
		break;
	}
}

static ssize_t do_read(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp;
	ssize_t result;

	if ((tp = line2tty(minor)) == NULL) {
		return ENXIO;
	}

	if (tp->tty_incaller != NONE || tp->tty_inleft > 0) {
		return EIO;
	}

	if (size == 0) {
		return EINVAL;
	}

	tp->tty_incaller = endpt;
	tp->tty_inid = id;
	tp->tty_ingrant = grant;
	tp->tty_inleft = size;
	tp->tty_incum = 0;

	if (!(tp->tty_termios.c_lflag & ICANON) && tp->tty_termios.c_cc[VTIME] > 0) {
		if (tp->tty_termios.c_cc[VMIN] == 0) {
			settimer(tp, TRUE);
			tp->tty_min = 1;
		} else {
			if (tp->tty_eotct == 0) {
				settimer(tp, FALSE);
				tp->tty_min = tp->tty_termios.c_cc[VMIN];
			}
		}
	}

	in_transfer(tp);
	handle_events(tp);
	
	if (tp->tty_inleft == 0) {
		return EDONTREPLY;
	}

	if (flags & CDEV_NONBLOCK) {
		tty_icancel(tp);
		result = (tp->tty_incum > 0) ? (ssize_t)tp->tty_incum : EAGAIN;
		tp->tty_inleft = 0;
		tp->tty_incum = 0;
		tp->tty_incaller = NONE;
		tp->tty_inid = 0;
		tp->tty_ingrant = 0;
		return result;
	}

	if (tp->tty_select_ops) {
		select_retry(tp);
	}

	return EDONTREPLY;
}

static ssize_t do_write(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp;

	if ((tp = line2tty(minor)) == NULL) {
		return ENXIO;
	}

	if (size == 0) {
		return EINVAL;
	}

	if (tp->tty_outcaller != NONE || tp->tty_outleft > 0) {
		return EIO;
	}

	tp->tty_outcaller = endpt;
	tp->tty_outid = id;
	tp->tty_outgrant = grant;
	assert(tp->tty_outcum == 0);
	tp->tty_outleft = size;

	handle_events(tp);

	if (tp->tty_outleft == 0) {
		return EDONTREPLY;
	}

	if (flags & CDEV_NONBLOCK) {
		ssize_t written_bytes = tp->tty_outcum;
		tp->tty_outleft = 0;
		tp->tty_outcum = 0;
		tp->tty_outcaller = NONE;
		return written_bytes > 0 ? written_bytes : EAGAIN;
	}

	if (tp->tty_select_ops) {
		select_retry(tp);
	}

	return EDONTREPLY;
}

static int handle_termios_get(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
    int r;

    if (tp == NULL) {
        return -EINVAL;
    }

    r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &tp->tty_termios,
                       sizeof(struct termios));

    return r;
}

static int handle_termios_set(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp,
                              unsigned long request)
{
	int r;
	
	if (request == TIOCSETAF) {
		tty_icancel(tp);
	}
		
	r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &tp->tty_termios,
		sizeof(struct termios));
	if (r != OK) {
		return r;
	}
		
	setattr(tp);
	return OK;
}

static int handle_flush(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	if (tp == NULL) {
		return -EFAULT; 
	}

	int flush_flags;
	int r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &flush_flags, sizeof(flush_flags));
	if (r != OK) {
		return r;
	}
		
	if (flush_flags & FREAD) {
		tty_icancel(tp);
	}
	
	if (flush_flags & FWRITE) {
		if (tp->tty_ocancel == NULL) {
            return -ENOSYS;
        }
		(*tp->tty_ocancel)(tp, 0);
	}
		
	return OK;
}

static int handle_winsize(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp,
                         unsigned long request)
{
    if (request == TIOCGWINSZ) {
        return sys_safecopyto(endpt, grant, 0, (vir_bytes) &tp->tty_winsize,
            sizeof(struct winsize));
    } else {
        int result = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &tp->tty_winsize,
            sizeof(struct winsize));
        
        if (result == OK) {
            sigchar(tp, SIGWINCH, 0);
        }
        
        return result;
    }
}

static int handle_bell(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	kio_bell_t bell;
	int r;
	
	if (!isconsole(tp)) {
		return OK;
	}
		
	r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &bell, sizeof(bell));
	if (r != OK) {
		return r;
	}
	
	long long usec_ticks_ll = (long long)bell.kb_duration.tv_usec * system_hz / 1000000LL;
	long long sec_ticks_ll = (long long)bell.kb_duration.tv_sec * system_hz;
	
	long long total_ticks_ll = usec_ticks_ll + sec_ticks_ll;

	if (total_ticks_ll == 0) {
		total_ticks_ll = 1;
	}
	
	clock_t final_ticks = (clock_t)total_ticks_ll;
		
	beep_x(bell.kb_pitch, final_ticks);
	
	return OK;
}

static int do_ioctl(devminor_t minor, unsigned long request, endpoint_t endpt,
	cp_grant_id_t grant, int flags, endpoint_t user_endpt, cdev_id_t id)
{
	tty_t *tp;
	int r = OK;

	if ((tp = line2tty(minor)) == NULL) {
		return ENXIO;
	}

	switch (request) {
	case TIOCGETA:
		r = handle_termios_get(endpt, grant, tp);
		break;

	case TIOCSETAW:
	case TIOCSETAF:
	case TIOCDRAIN:
		if (tp->tty_outleft > 0) {
			if (flags & CDEV_NONBLOCK) {
				return EAGAIN;
			}
			tp->tty_iocaller = endpt;
			tp->tty_ioid = id;
			tp->tty_ioreq = request;
			tp->tty_iogrant = grant;
			return EDONTREPLY;
		}
		if (request == TIOCDRAIN) {
			break;
		}
	case TIOCSETA:
		r = handle_termios_set(endpt, grant, tp, request);
		break;

	case TIOCFLUSH:
		r = handle_flush(endpt, grant, tp);
		break;

	case TIOCSTART:
		tp->tty_inhibited = 0;
		tp->tty_events = 1;
		break;

	case TIOCSTOP:
		tp->tty_inhibited = 1;
		tp->tty_events = 1;
		break;

	case TIOCSBRK:
		if (tp->tty_break_on != NULL) {
			(*tp->tty_break_on)(tp, 0);
		}
		break;

	case TIOCCBRK:
		if (tp->tty_break_off != NULL) {
			(*tp->tty_break_off)(tp, 0);
		}
		break;

	case TIOCGWINSZ:
	case TIOCSWINSZ:
		r = handle_winsize(endpt, grant, tp, request);
		break;

	case KIOCBELL:
		r = handle_bell(endpt, grant, tp);
		break;

	case TIOCGETD:
	{
		int i_val = TTYDISC;
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &i_val, sizeof(i_val));
		break;
	}

	case TIOCSETD:
		printf("TTY: TIOCSETD: can't set any other line discipline.\n");
		r = ENOTTY;
		break;

	case TIOCGLINED:
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes) lined, sizeof(lined));
		break;

	case TIOCGQSIZE:
	{
		int i_val = TTY_IN_BYTES;
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &i_val, sizeof(i_val));
		break;
	}

	case KIOCSMAP:
		if (isconsole(tp)) {
			r = kbd_loadmap(endpt, grant);
		} else {
			r = ENOTTY;
		}
		break;

	case TIOCSFON:
		if (isconsole(tp)) {
			r = con_loadfont(endpt, grant);
		} else {
			r = ENOTTY;
		}
		break;

	case TIOCSCTTY:
		tp->tty_pgrp = user_endpt;
		break;

	case TIOCGPGRP:
	case TIOCSPGRP:
	default:
		r = ENOTTY;
		break;
	}

	return r;
}

static int do_open(devminor_t minor, int access, endpoint_t user_endpt)
{
	tty_t *tp;
	int status = OK;

	tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	if (minor == LOG_MINOR && isconsole(tp)) {
		if (access & CDEV_R_BIT) {
			return EACCES;
		}
		return OK; /* No further actions for LOG_MINOR console if not read access */
	}

	if (!(access & CDEV_NOCTTY)) {
		tp->tty_pgrp = user_endpt;
		status = CDEV_CTTY;
	}

	tp->tty_openct++;
	if (tp->tty_openct == 1) {
		(*tp->tty_open)(tp, 0);
	}

	return status;
}

static int do_close(devminor_t minor)
{
	tty_t *tp;

	if ((tp = line2tty(minor)) == NULL) {
		return ENXIO;
	}

	if (minor != LOG_MINOR || !isconsole(tp)) {
		tp->tty_openct--;

		if (tp->tty_openct == 0) {
			tp->tty_pgrp = 0;
			tty_icancel(tp);
			(*tp->tty_ocancel)(tp, 0);
			(*tp->tty_close)(tp, 0);
			tp->tty_termios = termios_defaults;
			tp->tty_winsize = winsize_defaults;
			setattr(tp);
		}
	}

	return OK;
}

static int do_cancel(devminor_t minor, endpoint_t endpt, cdev_id_t id)
{
	tty_t *tp;

	if ((tp = line2tty(minor)) == NULL)
		return ENXIO;

	if (tp->tty_inleft != 0 && endpt == tp->tty_incaller && id == tp->tty_inid) {
		int result = tp->tty_incum > 0 ? tp->tty_incum : EAGAIN;
		tty_icancel(tp);
		tp->tty_inleft = 0;
		tp->tty_incum = 0;
		tp->tty_incaller = NONE;
		tp->tty_events = 1;
		return result;
	}

	if (tp->tty_outleft != 0 && endpt == tp->tty_outcaller &&
		id == tp->tty_outid) {
		int result = tp->tty_outcum > 0 ? tp->tty_outcum : EAGAIN;
		tp->tty_outleft = 0;
		tp->tty_outcum = 0;
		tp->tty_outcaller = NONE;
		tp->tty_events = 1;
		return result;
	}

	if (tp->tty_ioreq != 0 && endpt == tp->tty_iocaller &&
		id == tp->tty_ioid) {
		tp->tty_ioreq = 0;
		tp->tty_iocaller = NONE;
		tp->tty_events = 1;
		return EINTR;
	}
	
	return EDONTREPLY;
}

int select_try(struct tty *tp, int ops)
{
	if (tp == NULL) {
		return 0;
	}

	if (tp->tty_termios.c_ospeed == B0) {
		return ops;
	}

	int ready_ops = 0;

	if (ops & CDEV_OP_RD) {
		if (tp->tty_inleft > 0 ||
		    (tp->tty_incount > 0 && (!(tp->tty_termios.c_lflag & ICANON) || tp->tty_eotct > 0))) {
			ready_ops |= CDEV_OP_RD;
		}
	}

	if (ops & CDEV_OP_WR) {
		if (tp->tty_outleft > 0) {
			ready_ops |= CDEV_OP_WR;
		} else {
			if (tp->tty_devwrite != NULL && (*tp->tty_devwrite)(tp, 1)) {
				ready_ops |= CDEV_OP_WR;
			}
		}
	}
	
	return ready_ops;
}

int select_retry(struct tty *tp)
{
	int actual_ops = 0;

	if (tp->tty_select_ops != 0) {
		actual_ops = select_try(tp, tp->tty_select_ops);

		if (actual_ops != 0) {
			chardriver_reply_select(tp->tty_select_proc,
				tp->tty_select_minor, actual_ops);
			tp->tty_select_ops &= ~actual_ops;
		}
	}
	return OK;
}

static int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt)
{
	tty_t *tp;
	unsigned int requested_ops;
	unsigned int ready_ops;
	int watch_requested;

	if ((tp = line2tty(minor)) == NULL) {
		return ENXIO;
	}

	watch_requested = (ops & CDEV_NOTIFY);
	requested_ops = ops & (CDEV_OP_RD | CDEV_OP_WR | CDEV_OP_ERR);

	ready_ops = select_try(tp, requested_ops);

	unsigned int unready_ops = requested_ops & ~ready_ops;
	
	if (unready_ops && watch_requested) {
		if (tp->tty_select_ops != 0 && tp->tty_select_minor != minor) {
			printf("TTY: select on one object with two minors (%d, %d)\n",
				tp->tty_select_minor, minor);
			return EBADF;
		}
		tp->tty_select_ops |= unready_ops;
		tp->tty_select_proc = endpt;
		tp->tty_select_minor = minor;
	}

	return ready_ops;
}

void handle_events(tty_t *tp)
{
    if (tp == NULL) {
        return;
    }

    do {
        tp->tty_events = 0;

        if (tp->tty_devread != NULL) {
            (*tp->tty_devread)(tp, 0);
        }

        if (tp->tty_devwrite != NULL) {
            (*tp->tty_devwrite)(tp, 0);
        }
        
        if (tp->tty_ioreq != 0) {
            dev_ioctl(tp);
        }
    } while (tp->tty_events != 0);

    in_transfer(tp);

    if (tp->tty_incum >= tp->tty_min && tp->tty_inl != 0) {
        /* The original code was incomplete here, no body provided. */
    }
}