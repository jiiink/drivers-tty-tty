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
	for (tty_t *tp = FIRST_TTY; tp < END_TTY; tp++) {
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

    const int source = _ENDPOINT_P(tty_mess->m_source);

    switch (source) {
        case HARDWARE:
#if NR_RS_LINES > 0
            if (tty_mess->m_notify.interrupts & rs_irq_set) {
                rs_interrupt(tty_mess);
            }
#endif
            break;
        case CLOCK:
            break;
        default:
            return;
    }

    expire_timers(tty_mess->m_notify.timestamp);
}

static int handle_special_message(message *tty_mess, int ipc_status, int line)
{
    if (!IS_CDEV_RQ(tty_mess->m_type)) {
        switch (tty_mess->m_type) {
            case TTY_FKEY_CONTROL:
                do_fkey_ctl(tty_mess);
                break;
            case TTY_INPUT_UP:
            case TTY_INPUT_EVENT:
                do_input(tty_mess);
                break;
            default:
                chardriver_process(&tty_tab, tty_mess, ipc_status);
                break;
        }
        return 1;
    }

    if (line == VIDEO_MINOR) {
        do_video(tty_mess, ipc_status);
        return 1;
    }

    return 0;
}

static void process_received_message(void)
{
	message tty_mess;
	int ipc_status;
	int line;

	const int r = driver_receive(ANY, &tty_mess, &ipc_status);
	if (r != 0) {
		panic("driver_receive failed with: %d", r);
	}

	if (is_ipc_notify(ipc_status)) {
		handle_kernel_notification(&tty_mess, ipc_status);
		return;
	}

	if (OK != chardriver_get_minor(&tty_mess, &line)) {
		return;
	}

	if (handle_special_message(&tty_mess, ipc_status, line)) {
		return;
	}

	chardriver_process(&tty_tab, &tty_mess, ipc_status);
}

int main(void)
{
	sef_local_startup();

	while (TRUE) {
		process_tty_events();
		process_received_message();
	}
}

static void set_color(tty_t *tp, int color)
{
	char buf[16];
	int len;

	len = snprintf(buf, sizeof(buf), "\033[1;%dm", color);

	if (len > 0 && (size_t)len < sizeof(buf)) {
		do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)buf,
			(size_t)len, CDEV_NONBLOCK, 0);
	}
}

static void reset_color(tty_t *tp)
{
	static const char reset_sequence[] = "\033[0;39m";

	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)reset_sequence,
		 sizeof(reset_sequence) - 1, CDEV_NONBLOCK, 0);
}

#include <stddef.h>

// Assume these types and constants are defined elsewhere
typedef unsigned int devminor_t;
#define CONS_MINOR 0
#define LOG_MINOR 1
#define VIDEO_MINOR 2
#define RS232_MINOR 10
#define NR_CONS 4
#define NR_RS_LINES 2
extern devminor_t consoleline;

// Mock tty_t and related functions for compilation
typedef struct tty_s {
    int active;
    int id;
} tty_t;

tty_t *tty_addr(int index);
int tty_active(const tty_t *tp);

// Refactored function
tty_t *line2tty(devminor_t line)
{
	tty_t *tp;

	if (line == CONS_MINOR || line == LOG_MINOR) {
		line = consoleline;
	}

	if (line == VIDEO_MINOR) {
		return NULL;
	}

	if ((line - CONS_MINOR) < NR_CONS) {
		tp = tty_addr(line - CONS_MINOR);
	} else if ((line - RS232_MINOR) < NR_RS_LINES) {
		tp = tty_addr(line - RS232_MINOR + NR_CONS);
	} else {
		return NULL;
	}

	if (tp == NULL || !tty_active(tp)) {
		return NULL;
	}

	return tp;
}

static void sef_local_startup(void)
{
	int r;

	sef_setcb_init_fresh(sef_cb_init_fresh);
	sef_setcb_init_restart(SEF_CB_INIT_RESTART_STATEFUL);
	sef_setcb_signal_handler(sef_cb_signal_handler);

	r = sef_startup();

	/* sef_startup() should not return on success. */
	panic("sef_startup failed with error %d", r);
}

static int sef_cb_init_fresh(int type, sef_init_info_t *info)
{
	(void)type;
	(void)info;

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

	if ((r = tty_init()) != OK ||
	    (r = kb_init_once()) != OK ||
	    (r = sys_diagctl_register()) != OK) {
		return r;
	}

	return OK;
}

static void set_console_line(char term[CONS_ARG])
{
	if (strncmp(term, "console", CONS_ARG - 1) == 0) {
		consoleline = CONS_MINOR;
		return;
	}

	struct device_pattern {
		const char *format;
		int start_idx;
		int end_idx;
		int base_minor;
	};

	const struct device_pattern patterns[] = {
		{ "ttyc%d", 1, NR_CONS, CONS_MINOR },
		{ "tty0%d", 0, NR_RS_LINES, RS232_MINOR },
	};

	assert(NR_RS_LINES <= 9);

	for (size_t p = 0; p < sizeof(patterns) / sizeof(patterns[0]); ++p) {
		const struct device_pattern *pat = &patterns[p];
		for (int i = pat->start_idx; i < pat->end_idx; ++i) {
			char device_name[8];
			snprintf(device_name, sizeof(device_name), pat->format, i);

			const size_t max_dev_name_len = 5;
			size_t cmp_len = (CONS_ARG - 1 < max_dev_name_len)
				? CONS_ARG - 1 : max_dev_name_len;

			if (strncmp(term, device_name, cmp_len) == 0) {
				consoleline = pat->base_minor + i;
				return;
			}
		}
	}
}

#define SGR_COLOR_START 30
#define SGR_COLOR_END   37

static void set_kernel_color(const char *color)
{
	int color_offset = atoi(color);

	if (color_offset >= 0 && color_offset <= (SGR_COLOR_END - SGR_COLOR_START)) {
		kernel_msg_color = color_offset + SGR_COLOR_START;
	}
}

static void copy_kernel_messages(char *kernel_buf_copy, const struct kmessages *kmess_ptr,
                                 size_t prev_next, size_t bytes)
{
	const size_t len_to_end = _KMESS_BUF_SIZE - prev_next;
	const size_t part1_len = (len_to_end < bytes) ? len_to_end : bytes;
	const size_t part2_len = bytes - part1_len;

	memcpy(kernel_buf_copy, &kmess_ptr->km_buf[prev_next], part1_len);
	memcpy(kernel_buf_copy + part1_len, &kmess_ptr->km_buf[0], part2_len);
}

static void do_new_kmess(void)
{
	static int prev_next = 0;
	struct kmessages *kmess_ptr = get_minix_kerninfo()->kmessages;

	int next = kmess_ptr->km_next;
	int bytes = ((next + _KMESS_BUF_SIZE) - prev_next) % _KMESS_BUF_SIZE;

	if (bytes <= 0) {
		prev_next = next;
		return;
	}

	char kernel_buf_copy[_KMESS_BUF_SIZE];
	copy_kernel_messages(kernel_buf_copy, kmess_ptr, prev_next, next, bytes);

	tty_t *tp = line2tty(consoleline);
	if (tp == NULL) {
		panic("Don't know where to send kernel messages");
	}

	int saved_outleft = tp->tty_outleft;
	if (saved_outleft > 0) {
		tp->tty_outleft = 0;
	}

	if (kernel_msg_color != 0) {
		set_color(tp, kernel_msg_color);
	}

	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)kernel_buf_copy,
		 bytes, CDEV_NONBLOCK, 0);

	if (kernel_msg_color != 0) {
		reset_color(tp);
	}

	if (saved_outleft > 0) {
		tp->tty_outleft = saved_outleft;
	}

	prev_next = next;
}

static void sef_cb_signal_handler(int signo)
{
    switch (signo) {
        case SIGKMESS:
            do_new_kmess();
            break;
        case SIGTERM:
            cons_stop();
            break;
        default:
            /* Unhandled signal: ignore. */
            break;
    }
}

static void setup_read_timer(tty_t *tp)
{
	const struct termios *term = &tp->tty_termios;

	if ((term->c_lflag & ICANON) || term->c_cc[VTIME] == 0) {
		return;
	}

	if (term->c_cc[VMIN] == 0) {
		settimer(tp, TRUE);
		tp->tty_min = 1;
	} else if (tp->tty_eotct == 0) {
		settimer(tp, FALSE);
		tp->tty_min = term->c_cc[VMIN];
	}
}

static ssize_t do_read(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp = line2tty(minor);
	if (tp == NULL) {
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
	assert(tp->tty_incum == 0);

	setup_read_timer(tp);

	in_transfer(tp);
	handle_events(tp);

	if (tp->tty_inleft == 0) {
		return EDONTREPLY;
	}

	if (flags & CDEV_NONBLOCK) {
		ssize_t bytes_read = (ssize_t)tp->tty_incum;

		tty_icancel(tp);
		tp->tty_inleft = 0;
		tp->tty_incum = 0;
		tp->tty_incaller = NONE;

		return (bytes_read > 0) ? bytes_read : EAGAIN;
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
	tty_t *tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	if (tp->tty_outcaller != NONE || tp->tty_outleft > 0) {
		return EIO;
	}

	if (size == 0) {
		return EINVAL;
	}

	tp->tty_outcaller = endpt;
	tp->tty_outid = id;
	tp->tty_outgrant = grant;
	assert(tp->tty_outcum == 0);
	tp->tty_outleft = size;

	handle_events(tp);

	if ((flags & CDEV_NONBLOCK) == 0 || tp->tty_outleft == 0) {
		if (tp->tty_outleft > 0 && tp->tty_select_ops) {
			select_retry(tp);
		}
		return EDONTREPLY;
	}

	ssize_t bytes_written = tp->tty_outcum;
	ssize_t result = (bytes_written > 0) ? bytes_written : EAGAIN;

	tp->tty_outcaller = NONE;
	tp->tty_outleft = 0;
	tp->tty_outcum = 0;

	return result;
}

static int handle_termios_get(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	const vir_bytes_t grant_offset = 0;

	if (!tp) {
		return EINVAL;
	}

	return sys_safecopyto(endpt, grant, grant_offset,
		(vir_bytes)&tp->tty_termios, sizeof(tp->tty_termios));
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

    setattr(tp);
    return OK;
}

static int handle_flush(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	if (!tp) {
		return EINVAL;
	}

	int flush_flags;
	int status = sys_safecopyfrom(endpt, grant, 0,
		(vir_bytes)&flush_flags, sizeof(flush_flags));

	if (status != OK) {
		return status;
	}

	if (flush_flags & FREAD) {
		tty_icancel(tp);
	}

	if ((flush_flags & FWRITE) && tp->tty_ocancel) {
		tp->tty_ocancel(tp, 0);
	}

	return OK;
}

static int handle_winsize(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp,
                         unsigned long request)
{
	if (request == TIOCGWINSZ) {
		return sys_safecopyto(endpt, grant, 0, (vir_bytes) &tp->tty_winsize,
			sizeof(struct winsize));
	}

	int r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &tp->tty_winsize,
		sizeof(struct winsize));
	if (r == OK) {
		sigchar(tp, SIGWINCH, 0);
	}
		
	return r;
}

static int handle_bell(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	if (!isconsole(tp)) {
		return OK;
	}

	kio_bell_t bell;
	int r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&bell, sizeof(bell));
	if (r != OK) {
		return r;
	}

	const unsigned int usecs_per_sec = 1000000;
	clock_t ticks = (clock_t)bell.kb_duration.tv_sec * system_hz +
		      ((unsigned long long)bell.kb_duration.tv_usec * system_hz) / usecs_per_sec;

	if (ticks == 0) {
		ticks = 1;
	}

	beep_x(bell.kb_pitch, ticks);
	return OK;
}

static int do_ioctl(devminor_t minor, unsigned long request, endpoint_t endpt,
	cp_grant_id_t grant, int flags, endpoint_t user_endpt, cdev_id_t id)
{
	tty_t *tp;
	int r;

	tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	r = OK;
	switch (request) {
	case TIOCGETA:
		r = handle_termios_get(endpt, grant, tp);
		break;

	case TIOCSETA:
	case TIOCSETAW:
	case TIOCSETAF:
	case TIOCDRAIN:
		if (request != TIOCSETA && tp->tty_outleft > 0) {
			if ((flags & CDEV_NONBLOCK) != 0) {
				return EAGAIN;
			}
			tp->tty_iocaller = endpt;
			tp->tty_ioid = id;
			tp->tty_ioreq = request;
			tp->tty_iogrant = grant;
			return EDONTREPLY;
		}
		if (request != TIOCDRAIN) {
			r = handle_termios_set(endpt, grant, tp, request);
		}
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
		const int ldisc = TTYDISC;
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &ldisc, sizeof(ldisc));
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
		const int qsize = TTY_IN_BYTES;
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &qsize, sizeof(qsize));
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
	tty_t *tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	if (minor == LOG_MINOR && isconsole(tp)) {
		return (access & CDEV_R_BIT) ? EACCES : OK;
	}

	int r = OK;
	if (!(access & CDEV_NOCTTY)) {
		tp->tty_pgrp = user_endpt;
		r = CDEV_CTTY;
	}

	if (++tp->tty_openct == 1) {
		(*tp->tty_open)(tp, 0);
	}

	return r;
}

static int do_close(devminor_t minor)
{
	tty_t *tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	if (minor == LOG_MINOR && isconsole(tp)) {
		return OK;
	}

	if (--tp->tty_openct > 0) {
		return OK;
	}

	tp->tty_pgrp = 0;
	tty_icancel(tp);
	(*tp->tty_ocancel)(tp, 0);
	(*tp->tty_close)(tp, 0);
	tp->tty_termios = termios_defaults;
	tp->tty_winsize = winsize_defaults;
	setattr(tp);

	return OK;
}

static int handle_input_cancellation(tty_t *tp)
{
	int result;

	tty_icancel(tp);
	result = (tp->tty_incum > 0) ? tp->tty_incum : EAGAIN;
	tp->tty_inleft = tp->tty_incum = 0;
	tp->tty_incaller = NONE;
	return result;
}

static int handle_output_cancellation(tty_t *tp)
{
	int result;

	result = (tp->tty_outcum > 0) ? tp->tty_outcum : EAGAIN;
	tp->tty_outleft = tp->tty_outcum = 0;
	tp->tty_outcaller = NONE;
	return result;
}

static int handle_ioctl_cancellation(tty_t *tp)
{
	tp->tty_ioreq = 0;
	tp->tty_iocaller = NONE;
	return EINTR;
}

static int do_cancel(devminor_t minor, endpoint_t endpt, cdev_id_t id)
{
	tty_t *tp;
	int r = EDONTREPLY;

	tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	if (tp->tty_inleft != 0 && endpt == tp->tty_incaller && id == tp->tty_inid) {
		r = handle_input_cancellation(tp);
	} else if (tp->tty_outleft != 0 && endpt == tp->tty_outcaller &&
		   id == tp->tty_outid) {
		r = handle_output_cancellation(tp);
	} else if (tp->tty_ioreq != 0 && endpt == tp->tty_iocaller &&
		   id == tp->tty_ioid) {
		r = handle_ioctl_cancellation(tp);
	}

	if (r != EDONTREPLY) {
		tp->tty_events = 1;
	}

	return r;
}

int select_try(struct tty *tp, int ops)
{
	if (tp->tty_termios.c_ospeed == B0) {
		return ops;
	}

	int ready_ops = 0;

	const int device_is_readable =
	    (tp->tty_inleft > 0) ||
	    (tp->tty_incount > 0 &&
	     (!(tp->tty_termios.c_lflag & ICANON) || tp->tty_eotct > 0));

	if ((ops & CDEV_OP_RD) && device_is_readable) {
		ready_ops |= CDEV_OP_RD;
	}

	const int device_is_writable =
	    (tp->tty_outleft > 0) || ((*tp->tty_devwrite)(tp, 1) != 0);

	if ((ops & CDEV_OP_WR) && device_is_writable) {
		ready_ops |= CDEV_OP_WR;
	}

	return ready_ops;
}

int select_retry(struct tty *tp)
{
	if (!tp || !tp->tty_select_ops) {
		return OK;
	}

	int ops = select_try(tp, tp->tty_select_ops);
	if (ops) {
		chardriver_reply_select(tp->tty_select_proc,
			tp->tty_select_minor, ops);
		tp->tty_select_ops &= ~ops;
	}

	return OK;
}

static int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt)
{
	tty_t *tp = line2tty(minor);
	if (!tp) {
		return ENXIO;
	}

	const int watch = ops & CDEV_NOTIFY;
	const unsigned int requested_ops = ops & (CDEV_OP_RD | CDEV_OP_WR | CDEV_OP_ERR);
	const int ready_ops = select_try(tp, requested_ops);
	const unsigned int pending_ops = requested_ops & ~ready_ops;

	if (!pending_ops || !watch) {
		return ready_ops;
	}

	if (tp->tty_select_ops != 0 && tp->tty_select_minor != minor) {
		printf("TTY: select on one object with two minors (%d, %d)\n",
			tp->tty_select_minor, minor);
		return EBADF;
	}

	tp->tty_select_ops |= pending_ops;
	tp->tty_select_proc = endpt;
	tp->tty_select_minor = minor;

	return ready_ops;
}

static void process_device_io(tty_t *tp)
{
    do {
        tp->tty_events = 0;

        if (tp->tty_devread) {
            tp->tty_devread(tp, 0);
        }

        if (tp->tty_devwrite) {
            tp->tty_devwrite(tp, 0);
        }

        if (tp->tty_ioreq != 0) {
            dev_ioctl(tp);
        }
    } while (tp->tty_events != 0);
}

static void check_for_input_wakeup(tty_t *tp)
{
    if (tp->tty_incum >= tp->tty_min && tp->tty_inl > 0) {
        tty_wakeup(tp);
    }
}

void handle_events(tty_t *tp)
{
    if (!tp) {
        return;
    }

    process_device_io(tp);

    in_transfer(tp);

    check_for_input_wakeup(tp);
}