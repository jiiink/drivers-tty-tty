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

	if (FIRST_TTY == NULL || END_TTY == NULL || FIRST_TTY >= END_TTY) {
		return;
	}

	for (tp = FIRST_TTY; tp < END_TTY; tp++) {
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

	endpoint_t source_endpoint = _ENDPOINT_P(tty_mess->m_source);
	_Bool expire_timers_needed = 0; // Use _Bool for compatibility without stdbool.h

	switch (source_endpoint) {
	case CLOCK:
		expire_timers_needed = 1;
		break;
	case HARDWARE:
#if NR_RS_LINES > 0
		if (tty_mess->m_notify.interrupts & rs_irq_set) {
			rs_interrupt(tty_mess);
		}
#endif
		expire_timers_needed = 1;
		break;
	default:
		break;
	}

	if (expire_timers_needed) {
		expire_timers(tty_mess->m_notify.timestamp);
	}
}

static int handle_special_message(message *tty_mess, int ipc_status, int line)
{
	if (tty_mess == NULL) {
		return 0;
	}

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
	}
	
	if (line == VIDEO_MINOR) {
		do_video(tty_mess, ipc_status);
		return 1;
	}
	
	return 0;
}

int main(void)
{
	message tty_mess;
	int ipc_status;
	int line;

	sef_local_startup();
	
	while (TRUE) {
		process_tty_events();

		int r = driver_receive(ANY, &tty_mess, &ipc_status);
		if (r != 0)
			panic("driver_receive failed with: %d", r);

		if (is_ipc_notify(ipc_status)) {
			handle_kernel_notification(&tty_mess, ipc_status);
			continue;
		}
		
		if (OK != chardriver_get_minor(&tty_mess, &line))
			continue;
			
		if (handle_special_message(&tty_mess, ipc_status, line))
			continue;

		chardriver_process(&tty_tab, &tty_mess, ipc_status);
	}

	return 0;
}

static void set_color(tty_t *tp, int color)
{
    char buf[10];
    int chars_formatted;
    size_t write_len;

    buf[0] = '\033';

    chars_formatted = snprintf(&buf[1], sizeof(buf) - 1, "[1;%dm", color);

    if (chars_formatted < 0 || (size_t)chars_formatted >= sizeof(buf) - 1) {
        return;
    }

    write_len = 1 + chars_formatted;

    if (do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)buf, write_len,
                 CDEV_NONBLOCK, 0) < 0) {
        
    }
}

static const int SGR_COLOR_RESET_CODE = 39;
static const char ESC_CHAR = '\033';

static void reset_color(tty_t *tp)
{
    char buf[8];
    int formatted_len;

    buf[0] = ESC_CHAR;
    formatted_len = snprintf(&buf[1], sizeof(buf) - 1, "[0;%dm", SGR_COLOR_RESET_CODE);

    if (formatted_len < 0 || (size_t)formatted_len >= sizeof(buf) - 1) {
        return;
    }

    size_t total_write_size = 1 + (size_t)formatted_len;

    int write_result = do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) buf, total_write_size,
                                CDEV_NONBLOCK, 0);

    if (write_result != 0) {
        // Error handling for do_write. No action taken as per constraints.
    }
}

tty_t *line2tty(devminor_t line)
{
	if (line == CONS_MINOR || line == LOG_MINOR) {
		line = consoleline;
	}

	if (line == VIDEO_MINOR) {
		return NULL;
	}

	tty_t *tp = NULL;

	if (line >= CONS_MINOR && (line - CONS_MINOR) < NR_CONS) {
		tp = tty_addr((int)(line - CONS_MINOR));
	} else if (line >= RS232_MINOR && (line - RS232_MINOR) < NR_RS_LINES) {
		tp = tty_addr((int)(line - RS232_MINOR + NR_CONS));
	}

	if (tp != NULL && !tty_active(tp)) {
		tp = NULL;
	}

	return tp;
}

static void sef_local_startup(void)
{
	int status;

	status = sef_setcb_init_fresh(sef_cb_init_fresh);
	if (status != 0) {
		return;
	}

	status = sef_setcb_init_restart(SEF_CB_INIT_RESTART_STATEFUL);
	if (status != 0) {
		return;
	}

	status = sef_setcb_signal_handler(sef_cb_signal_handler);
	if (status != 0) {
		return;
	}

	status = sef_startup();
	if (status != 0) {
		return;
	}
}

static int sef_cb_init_fresh(int UNUSED(type), sef_init_info_t *UNUSED(info))
{
	int r;
	char val[CONS_ARG];

	r = sys_getmachine(&machine);
	if (r != OK) {
		panic("Couldn't obtain kernel environment: %d", r);
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

#include <string.h> // For strncmp, strlcpy

// Helper function for safe string comparison length.
// It ensures that `strncmp` does not read beyond the bounds of `term` (up to `max_term_buffer_len`)
// and does not compare beyond the actual length of the `pattern` (up to `pattern_len`).
// `max_term_buffer_len` should be `CONS_ARG - 1`.
static inline size_t get_safe_cmp_len(size_t max_term_buffer_len, size_t pattern_len)
{
    // The comparison length is capped by the maximum number of characters available in the input 'term' buffer
    // (excluding the null terminator, hence `max_term_buffer_len`)
    // and by the actual length of the pattern string.
    return (max_term_buffer_len < pattern_len) ? max_term_buffer_len : pattern_len;
}

static void set_console_line(char term[CONS_ARG])
{
    // 1. Check for the exact "console" string.
    const char *console_pattern = "console";
    const size_t console_pattern_actual_len = strlen(console_pattern);
    if (!strncmp(term, console_pattern, get_safe_cmp_len(CONS_ARG - 1, console_pattern_actual_len))) {
        consoleline = CONS_MINOR;
        return;
    }

    // 2. Check for "ttycX" patterns (e.g., "ttyc1", "ttyc2", ...)
    // Base string "ttyc0" is 5 characters long.
    const char ttyc_base_pattern[] = "ttyc0";
    const size_t ttyc_pattern_actual_len = sizeof(ttyc_base_pattern) - 1; // 5 characters ("ttyc0")
    char current_ttyc_name[sizeof(ttyc_base_pattern)]; // Buffer for "ttycX\0" (6 bytes)

    for (int i = 1; i < NR_CONS; i++) {
        // Construct "ttycX" string. The last character of the pattern (index 4) is the digit.
        strlcpy(current_ttyc_name, ttyc_base_pattern, sizeof(current_ttyc_name));
        current_ttyc_name[ttyc_pattern_actual_len - 1] = '0' + i; // Changes '0' to '1', '2', etc.
        // strlcpy ensures null termination up to sizeof(current_ttyc_name)-1.
        // Since `ttyc_base_pattern` ("ttyc0") includes a null, and we only modify one character
        // before the original null, the string remains null-terminated.

        if (!strncmp(term, current_ttyc_name, get_safe_cmp_len(CONS_ARG - 1, ttyc_pattern_actual_len))) {
            consoleline = CONS_MINOR + i;
            return;
        }
    }

    // 3. Check for "tty0X" patterns (e.g., "tty00", "tty01", ...)
    // Base string "tty00" is 5 characters long.
    // The original code implies NR_RS_LINES <= 9, ensuring '0' + i remains a single digit.
    const char tty0_base_pattern[] = "tty00";
    const size_t tty0_pattern_actual_len = sizeof(tty0_base_pattern) - 1; // 5 characters ("tty00")
    char current_tty0_name[sizeof(tty0_base_pattern)]; // Buffer for "tty0X\0" (6 bytes)

    for (int i = 0; i < NR_RS_LINES; i++) {
        // Construct "tty0X" string. The last character of the pattern (index 4) is the digit.
        strlcpy(current_tty0_name, tty0_base_pattern, sizeof(current_tty0_name));
        current_tty0_name[tty0_pattern_actual_len - 1] = '0' + i; // Changes '0' to '0', '1', etc.
        // Null termination handled as described for ttyc_name.

        if (!strncmp(term, current_tty0_name, get_safe_cmp_len(CONS_ARG - 1, tty0_pattern_actual_len))) {
            consoleline = RS232_MINOR + i;
            return;
        }
    }
}

#include <stdlib.h>
#include <errno.h>

static const int SGR_COLOR_START_VAL = 30;
static const int SGR_COLOR_END_VAL = 37;

static void set_kernel_color(char color[CONS_ARG])
{
    long parsed_value;
    char *endptr;

    errno = 0;

    parsed_value = strtol(color, &endptr, 10);

    if (color == endptr || *endptr != '\0' || errno == ERANGE) {
        return;
    }

    int def_color = (int)parsed_value;

    if (def_color >= 0 && def_color <= (SGR_COLOR_END_VAL - SGR_COLOR_START_VAL)) {
        kernel_msg_color = def_color + SGR_COLOR_START_VAL;
    }
}

static void copy_kernel_messages(char *kernel_buf_copy, struct kmessages *kmess_ptr, int prev_next, int bytes)
{
    if (kernel_buf_copy == NULL || kmess_ptr == NULL) {
        return;
    }
    if (bytes <= 0) {
        return;
    }
    if (prev_next < 0 || prev_next >= _KMESS_BUF_SIZE) {
        return;
    }

    size_t bytes_to_copy = (size_t)bytes;
    size_t current_offset = (size_t)prev_next;

    size_t remaining_in_buffer_from_offset = (size_t)_KMESS_BUF_SIZE - current_offset;

    size_t first_segment_len;
    if (remaining_in_buffer_from_offset < bytes_to_copy) {
        first_segment_len = remaining_in_buffer_from_offset;
    } else {
        first_segment_len = bytes_to_copy;
    }

    memcpy(kernel_buf_copy, &kmess_ptr->km_buf[current_offset], first_segment_len);

    if (first_segment_len < bytes_to_copy) {
        memcpy(&kernel_buf_copy[first_segment_len], &kmess_ptr->km_buf[0], bytes_to_copy - first_segment_len);
    }
}

static void do_new_kmess(void)
{
	struct kmessages *kmess_ptr;
	char kernel_buf_copy[_KMESS_BUF_SIZE];
	static int prev_next = 0;
	int next_idx, bytes_to_copy;
	tty_t *tty_ptr;
	int original_tty_outleft = -1;

	kmess_ptr = get_minix_kerninfo()->kmessages;
	next_idx = kmess_ptr->km_next;

	bytes_to_copy = (next_idx - prev_next + _KMESS_BUF_SIZE) % _KMESS_BUF_SIZE;
	
	if (bytes_to_copy == 0) {
		prev_next = next_idx;
		return;
	}

	copy_kernel_messages(kernel_buf_copy, kmess_ptr, prev_next, next_idx, bytes_to_copy);

	tty_ptr = line2tty(consoleline);
	if (tty_ptr == NULL) {
		panic("Don't know where to send kernel messages");
	}
		
	if (tty_ptr->tty_outleft > 0) {
		original_tty_outleft = tty_ptr->tty_outleft;
		tty_ptr->tty_outleft = 0;
	}

	if (kernel_msg_color != 0) {
		set_color(tty_ptr, kernel_msg_color);
	}
		
	do_write(tty_ptr->tty_minor, 0, KERNEL, (cp_grant_id_t) kernel_buf_copy, 
	         bytes_to_copy, CDEV_NONBLOCK, 0);
	         
	if (kernel_msg_color != 0) {
		reset_color(tty_ptr);
	}
		
	if (original_tty_outleft != -1) {
		tty_ptr->tty_outleft = original_tty_outleft;
	}

	prev_next = next_idx;
}

static void sef_cb_signal_handler(int signo)
{
	switch(signo) {
	case SIGKMESS:
		do_new_kmess();
		break;
	case SIGTERM:
		cons_stop();
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
	ssize_t r;

	if ((tp = line2tty(minor)) == NULL) {
		return ENXIO;
	}

	if (tp->tty_incaller != NONE || tp->tty_inleft > 0) {
		return EIO;
	}

	if (size <= 0) {
		return EINVAL;
	}

	tp->tty_incaller = endpt;
	tp->tty_inid = id;
	tp->tty_ingrant = grant;
	assert(tp->tty_incum == 0);
	tp->tty_inleft = size;

	if (!(tp->tty_termios.c_lflag & ICANON) && tp->tty_termios.c_cc[VTIME] > 0) {
		if (tp->tty_termios.c_cc[VMIN] == 0) {
			settimer(tp, TRUE);
			tp->tty_min = 1;
		} else if (tp->tty_eotct == 0) {
			settimer(tp, FALSE);
			tp->tty_min = tp->tty_termios.c_cc[VMIN];
		}
	}

	in_transfer(tp);
	handle_events(tp);
	
	if (tp->tty_inleft == 0) {
		return EDONTREPLY;
	}

	if (flags & CDEV_NONBLOCK) {
		tty_icancel(tp);
		r = tp->tty_incum > 0 ? tp->tty_incum : EAGAIN;
		tp->tty_inleft = 0;
		tp->tty_incum = 0;
		tp->tty_incaller = NONE;
		return r;
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
	ssize_t result;

	if (size == 0) {
		return EINVAL;
	}

	tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	if (tp->tty_outcaller != NONE || tp->tty_outleft > 0) {
		return EIO;
	}

	assert(tp->tty_outcum == 0);

	tp->tty_outcaller = endpt;
	tp->tty_outid = id;
	tp->tty_outgrant = grant;
	tp->tty_outleft = size;

	handle_events(tp);

	if (flags & CDEV_NONBLOCK) {
		if (tp->tty_outcum > 0) {
			result = tp->tty_outcum;
		} else {
			result = EAGAIN;
		}

		tp->tty_outcaller = NONE;
		tp->tty_outid = 0;
		tp->tty_outgrant = 0;
		tp->tty_outleft = 0;
		tp->tty_outcum = 0;
	} else {
		if (tp->tty_outleft == 0) {
			result = EDONTREPLY;

			tp->tty_outcaller = NONE;
			tp->tty_outid = 0;
			tp->tty_outgrant = 0;
			tp->tty_outleft = 0;
			tp->tty_outcum = 0;
		} else {
			result = EDONTREPLY;
			if (tp->tty_select_ops) {
				select_retry(tp);
			}
		}
	}

	return result;
}

static int handle_termios_get(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	if (tp == NULL) {
		return EINVAL;
	}

	return sys_safecopyto(endpt, grant, 0, (vir_bytes) &tp->tty_termios,
		sizeof(struct termios));
}

static int handle_termios_set(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp,
                              unsigned long request)
{
	if (request == TIOCSETAF) {
		tty_icancel(tp);
	}

	int r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &tp->tty_termios,
		sizeof(struct termios));
	if (r != OK) {
		return r;
	}

	/* Assuming setattr returns an int status code, consistent with sys_safecopyfrom */
	r = setattr(tp);
	if (r != OK) {
		return r;
	}

	return OK;
}

static int handle_flush(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	if (tp == NULL) {
		return EINVAL;
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
		(*tp->tty_ocancel)(tp, 0);
	}
		
	return OK;
}

static int handle_winsize(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp,
                         unsigned long request)
{
	int r;
	
	if (request == TIOCGWINSZ) {
		return sys_safecopyto(endpt, grant, 0, (vir_bytes) &tp->tty_winsize,
			sizeof(struct winsize));
	}
	
	r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &tp->tty_winsize,
		sizeof(struct winsize));
	if (r == OK) {
		sigchar(tp, SIGWINCH, 0);
	}
		
	return r;
}

static int handle_bell(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	kio_bell_t bell;
	
	if (!isconsole(tp)) {
		return OK;
	}
		
	int r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &bell, sizeof(bell));
	if (r != OK) {
		return r;
	}
		
	clock_t ticks = ((long long)bell.kb_duration.tv_usec * system_hz) / 1000000LL;
	ticks += (long long)bell.kb_duration.tv_sec * system_hz;
	
	if (ticks == 0) {
		ticks = 1;
	}
	
	if (ticks > (5 * system_hz)) {
		ticks = (5 * system_hz);
	}
		
	beep_x(bell.kb_pitch, ticks);
	
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
					r = EAGAIN;
				} else {
					tp->tty_iocaller = endpt;
					tp->tty_ioid = id;
					tp->tty_ioreq = request;
					tp->tty_iogrant = grant;
					r = EDONTREPLY;
				}
			} else { /* tp->tty_outleft == 0 */
				if (request == TIOCDRAIN) {
					/* TIOCDRAIN is a no-op if output buffer is empty. */
					r = OK;
				} else { /* TIOCSETAW or TIOCSETAF */
					r = handle_termios_set(endpt, grant, tp, request);
				}
			}
			break;

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

		case TIOCGETD: {
			const int discipline_id = TTYDISC;
			r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &discipline_id, sizeof(discipline_id));
			break;
		}

		case TIOCSETD:
			r = ENOTTY; /* Cannot set any other line discipline. */
			break;

		case TIOCGLINED:
			r = sys_safecopyto(endpt, grant, 0, (vir_bytes) lined, sizeof(lined));
			break;

		case TIOCGQSIZE: {
			const int queue_size = TTY_IN_BYTES;
			r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &queue_size, sizeof(queue_size));
			break;
		}

		case KIOCSMAP:
			if (isconsole(tp)) {
				r = kbd_loadmap(endpt, grant);
			}
			break;

		case TIOCSFON:
			if (isconsole(tp)) {
				r = con_loadfont(endpt, grant);
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
    int result = OK;

    tp = line2tty(minor);
    if (tp == NULL) {
        return ENXIO;
    }

    if (minor == LOG_MINOR && isconsole(tp)) {
        if (access & CDEV_R_BIT) {
            return EACCES;
        }
        return OK;
    }

    if (!(access & CDEV_NOCTTY)) {
        tp->tty_pgrp = user_endpt;
        result = CDEV_CTTY;
    }

    tp->tty_openct++;

    if (tp->tty_openct == 1) {
        if (tp->tty_open != NULL) {
            (*tp->tty_open)(tp, 0);
        }
    }

    return result;
}

static int do_close(devminor_t minor)
{
	tty_t *tp;

	if ((tp = line2tty(minor)) == NULL) {
		return ENXIO;
	}

	int remaining_openct = --tp->tty_openct;

	int is_last_close = (remaining_openct == 0);
	int is_not_special_log_console = (minor != LOG_MINOR || !isconsole(tp));

	if (is_last_close && is_not_special_log_console) {
		tp->tty_pgrp = 0;
		tty_icancel(tp);
		(*tp->tty_ocancel)(tp, 0);
		(*tp->tty_close)(tp, 0);
		tp->tty_termios = termios_defaults;
		tp->tty_winsize = winsize_defaults;
		setattr(tp);
	}

	return OK;
}

static int do_cancel(devminor_t minor, endpoint_t endpt, cdev_id_t id)
{
	tty_t *tp;
	int r;

	if ((tp = line2tty(minor)) == NULL) {
		return ENXIO;
	}

	r = EDONTREPLY;

	if (tp->tty_inleft != 0 && endpt == tp->tty_incaller && id == tp->tty_inid) {
		tty_icancel(tp);
		r = (tp->tty_incum > 0) ? tp->tty_incum : EAGAIN;
		tp->tty_inleft = 0;
		tp->tty_incum = 0;
		tp->tty_incaller = NONE;
	} else if (tp->tty_outleft != 0 && endpt == tp->tty_outcaller && id == tp->tty_outid) {
		r = (tp->tty_outcum > 0) ? tp->tty_outcum : EAGAIN;
		tp->tty_outleft = 0;
		tp->tty_outcum = 0;
		tp->tty_outcaller = NONE;
	} else if (tp->tty_ioreq != 0 && endpt == tp->tty_iocaller && id == tp->tty_ioid) {
		r = EINTR;
		tp->tty_ioreq = 0;
		tp->tty_iocaller = NONE;
	}
	
	if (r != EDONTREPLY) {
		tp->tty_events = 1;
	}
		
	return r;
}

int select_try(struct tty *tp, int ops)
{
	int ready_ops = 0;

	if (tp == NULL) {
		return 0;
	}

	if (tp->tty_termios.c_ospeed == B0) {
		ready_ops |= ops;
	}

	if (ops & CDEV_OP_RD) {
		int read_available = 0;

		if (tp->tty_inleft > 0) {
			read_available = 1;
		} else if (tp->tty_incount > 0) {
			if (!(tp->tty_termios.c_lflag & ICANON) || tp->tty_eotct > 0) {
				read_available = 1;
			}
		}

		if (read_available) {
			ready_ops |= CDEV_OP_RD;
		}
	}

	if (ops & CDEV_OP_WR) {
		int write_available = 0;

		if (tp->tty_outleft > 0) {
			write_available = 1;
		} else if (tp->tty_devwrite != NULL) {
			if ((*tp->tty_devwrite)(tp, 1)) {
				write_available = 1;
			}
		}

		if (write_available) {
			ready_ops |= CDEV_OP_WR;
		}
	}
	
	return ready_ops;
}

int select_retry(struct tty *tp)
{
    int requested_ops;
    int available_ops;

    requested_ops = tp->tty_select_ops;
    if (requested_ops == 0) {
        return OK;
    }

    available_ops = select_try(tp, requested_ops);

    if (available_ops < 0) {
        // An error occurred in select_try.
        // According to the original function's contract, we must still return OK.
        // No operations are processed, and the pending ops mask remains unchanged.
        return OK;
    }

    if (available_ops > 0) {
        chardriver_reply_select(tp->tty_select_proc, tp->tty_select_minor, available_ops);
        tp->tty_select_ops &= ~available_ops;
    }

    return OK;
}

static int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt)
{
	tty_t *tp;
	unsigned int requested_ops_all = ops;
	unsigned int filtered_ops;
	unsigned int ready_ops;
	unsigned int pending_ops;
	int is_watch_requested;

	if ((tp = line2tty(minor)) == NULL)
		return ENXIO;

	is_watch_requested = (requested_ops_all & CDEV_NOTIFY) != 0;

	filtered_ops = requested_ops_all & (CDEV_OP_RD | CDEV_OP_WR | CDEV_OP_ERR);

	ready_ops = select_try(tp, filtered_ops);

	pending_ops = filtered_ops & ~ready_ops;
	
	if (pending_ops && is_watch_requested) {
		if (tp->tty_select_ops != 0 && tp->tty_select_minor != minor) {
			printf("TTY: select on one object with two minors (%d, %d)\n",
				tp->tty_select_minor, minor);
			return EBADF;
		}
		
		tp->tty_select_ops |= pending_ops;
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

	if (tp->tty_devread == NULL || tp->tty_devwrite == NULL) {
		return;
	}

	do {
		tp->tty_events = 0;
		(*tp->tty_devread)(tp, 0);
		(*tp->tty_devwrite)(tp, 0);
		if (tp->tty_ioreq)
			dev_ioctl(tp);
	} while (tp->tty_events);

	in_transfer(tp);

	if (tp->tty_incum >= tp->tty_min && tp->tty_inl