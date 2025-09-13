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
	tty_t *start = FIRST_TTY;
	tty_t *end = END_TTY;
	tty_t *tp;

	if (start == NULL || end == NULL || start >= end) {
		return;
	}

	for (tp = start; tp < end; ++tp) {
		if (tp->tty_events != 0) {
			handle_events(tp);
		}
	}
}

static void handle_kernel_notification(message *tty_mess, int ipc_status)
{
	int source;
	int should_expire = 0;

	if (tty_mess == NULL || !is_ipc_notify(ipc_status))
		return;

	source = _ENDPOINT_P(tty_mess->m_source);
	switch (source) {
	case CLOCK:
		should_expire = 1;
		break;
	case HARDWARE:
#if NR_RS_LINES > 0
		if ((tty_mess->m_notify.interrupts & rs_irq_set) != 0) {
			rs_interrupt(tty_mess);
		}
#endif
		should_expire = 1;
		break;
	default:
		break;
	}

	if (should_expire) {
		expire_timers(tty_mess->m_notify.timestamp);
	}
}

static int handle_special_message(message *tty_mess, int ipc_status, int line)
{
	if (tty_mess == NULL) {
		return 0;
	}

	int type = tty_mess->m_type;

	switch (type) {
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

	if (IS_CDEV_RQ(type)) {
		if (line == VIDEO_MINOR) {
			do_video(tty_mess, ipc_status);
			return 1;
		}
		return 0;
	}

	chardriver_process(&tty_tab, tty_mess, ipc_status);
	return 1;
}

int main(void)
{
	message tty_mess;
	int ipc_status;
	int line = -1;
	int recv_rc;

	sef_local_startup();

	for (;;) {
		process_tty_events();

		recv_rc = driver_receive(ANY, &tty_mess, &ipc_status);
		if (recv_rc != 0) {
			panic("driver_receive failed with: %d", recv_rc);
		}

		if (is_ipc_notify(ipc_status)) {
			handle_kernel_notification(&tty_mess, ipc_status);
			continue;
		}

		if (chardriver_get_minor(&tty_mess, &line) != OK) {
			continue;
		}

		if (handle_special_message(&tty_mess, ipc_status, line)) {
			continue;
		}

		chardriver_process(&tty_tab, &tty_mess, ipc_status);
	}

	return 0;
}

static void set_color(tty_t *tp, int color)
{
	char buf[8];
	int r;

	if (tp == NULL) {
		return;
	}

	memset(buf, 0, sizeof(buf));
	r = snprintf(buf, sizeof(buf), "\033[1;%dm", color);
	if (r < 0) {
		return;
	}

	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) buf, sizeof(buf),
		CDEV_NONBLOCK, 0);
}

static void reset_color(tty_t *tp)
{
	static const char seq[] = "\033[0;39m";
	if (tp == NULL) {
		return;
	}
	(void)do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)seq, sizeof(seq) - 1, CDEV_NONBLOCK, 0);
}

tty_t *line2tty(devminor_t line)
{
	if (line == CONS_MINOR || line == LOG_MINOR) {
		line = consoleline;
	}

	if (line == VIDEO_MINOR) {
		return NULL;
	}

	unsigned long long l = (unsigned long long) line;
	const unsigned long long cmin = (unsigned long long) CONS_MINOR;
	const unsigned long long rmin = (unsigned long long) RS232_MINOR;
	const unsigned long long ncons = (unsigned long long) NR_CONS;
	const unsigned long long nrs = (unsigned long long) NR_RS_LINES;

	tty_t *tp = NULL;

	if (l >= cmin && l < cmin + ncons) {
		tp = tty_addr((size_t) (l - cmin));
	} else if (l >= rmin && l < rmin + nrs) {
		tp = tty_addr((size_t) (l - rmin + ncons));
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
    sef_setcb_init_fresh(sef_cb_init_fresh);
    sef_setcb_init_restart(SEF_CB_INIT_RESTART_STATEFUL);
    sef_setcb_signal_handler(sef_cb_signal_handler);
    sef_startup();
}

static int fetch_env_string(const char *name, char *dst, size_t dst_size)
{
	int r;

	if (name == NULL || dst == NULL || dst_size == 0) {
		return -1;
	}

	r = env_get_param(name, dst, dst_size);
	if (r == OK) {
		dst[dst_size - 1] = '\0';
	}

	return r;
}

static int sef_cb_init_fresh(int UNUSED(type), sef_init_info_t *UNUSED(info))
{
	char val[CONS_ARG];
	int r;

	r = sys_getmachine(&machine);
	if (r != OK) {
		panic("Couldn't obtain kernel environment: %d", r);
	}

	if (fetch_env_string("console", val, sizeof(val)) == OK) {
		set_console_line(val);
	}

	if (fetch_env_string("kernelclr", val, sizeof(val)) == OK) {
		set_kernel_color(val);
	}

	tty_init();
	kb_init_once();
	(void)sys_diagctl_register();

	return OK;
}

static void set_console_line(char term[CONS_ARG])
{
	size_t maxlen = CONS_ARG > 0 ? (size_t)CONS_ARG - 1 : 0;
	size_t len = strnlen(term, maxlen);

	if (len == 7 && !strncmp(term, "console", 7)) {
		consoleline = CONS_MINOR;
		return;
	}

	if (len >= 5 && !strncmp(term, "ttyc", 4)) {
		char d = term[4];
		if (d >= '1' && d <= '9') {
			int i = d - '0';
			if (i < NR_CONS) {
				consoleline = CONS_MINOR + i;
				return;
			}
		}
	}

	assert(NR_RS_LINES <= 9);
	if (len >= 5 && !strncmp(term, "tty0", 4)) {
		char d = term[4];
		if (d >= '0' && d <= '9') {
			int i = d - '0';
			if (i < NR_RS_LINES) {
				consoleline = RS232_MINOR + i;
				return;
			}
		}
	}
}

static void set_kernel_color(char color[CONS_ARG])
{
	long def_color;
	char *endptr;

#define SGR_COLOR_START 30
#define SGR_COLOR_END 37

	if (color == NULL) {
		return;
	}

	errno = 0;
	def_color = strtol(color, &endptr, 10);
	if (errno == ERANGE) {
		return;
	}

	if (def_color >= 0 && def_color <= (SGR_COLOR_END - SGR_COLOR_START)) {
		kernel_msg_color = (int)(def_color + SGR_COLOR_START);
	}
}

static void copy_kernel_messages(char *kernel_buf_copy, struct kmessages *kmess_ptr, 
                                 int prev_next, int next, int bytes)
{
	size_t start, to_copy, first_chunk, second_chunk;
	const size_t bufsize = _KMESS_BUF_SIZE;

	(void)next;

	if (kernel_buf_copy == NULL || kmess_ptr == NULL || bytes <= 0) {
		return;
	}

	to_copy = (size_t)bytes;
	if (to_copy > bufsize) {
		to_copy = bufsize;
	}

	if (prev_next < 0) {
		start = 0;
	} else {
		start = (size_t)prev_next;
		if (start >= bufsize) {
			start %= bufsize;
		}
	}

	first_chunk = bufsize - start;
	if (first_chunk > to_copy) {
		first_chunk = to_copy;
	}

	memcpy(kernel_buf_copy, &kmess_ptr->km_buf[start], first_chunk);

	second_chunk = to_copy - first_chunk;
	if (second_chunk > 0) {
		memcpy(kernel_buf_copy + first_chunk, kmess_ptr->km_buf, second_chunk);
	}
}

static void do_new_kmess(void)
{
	static int prev_next = 0;
	struct kmessages *kmess_ptr;
	char kernel_buf_copy[_KMESS_BUF_SIZE];
	int next, bytes;
	tty_t *tp, rtp;
	int restore = 0;
	int color;

	kmess_ptr = get_minix_kerninfo()->kmessages;
	next = kmess_ptr->km_next;
	bytes = ((next + _KMESS_BUF_SIZE) - prev_next) % _KMESS_BUF_SIZE;

	if (bytes <= 0) {
		prev_next = next;
		return;
	}

	copy_kernel_messages(kernel_buf_copy, kmess_ptr, prev_next, next, bytes);

	tp = line2tty(consoleline);
	if (tp == NULL)
		panic("Don't know where to send kernel messages");

	if (tp->tty_outleft > 0) {
		rtp = *tp;
		tp->tty_outleft = 0;
		restore = 1;
	}

	color = kernel_msg_color;
	if (color != 0)
		set_color(tp, color);

	(void)do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)kernel_buf_copy,
	               bytes, CDEV_NONBLOCK, 0);

	if (color != 0)
		reset_color(tp);

	if (restore) {
		*tp = rtp;
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
		break;
	}
}

static ssize_t do_read(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp = line2tty(minor);

	if (tp == NULL)
		return ENXIO;

	if (tp->tty_incaller != NONE || tp->tty_inleft > 0)
		return EIO;

	if (size <= 0)
		return EINVAL;

	tp->tty_incaller = endpt;
	tp->tty_inid = id;
	tp->tty_ingrant = grant;
	assert(tp->tty_incum == 0);
	tp->tty_inleft = size;

	if (!(tp->tty_termios.c_lflag & ICANON) &&
	    tp->tty_termios.c_cc[VTIME] > 0) {
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

	if (tp->tty_inleft == 0)
		return EDONTREPLY;

	if (flags & CDEV_NONBLOCK) {
		int r;
		tty_icancel(tp);
		r = (tp->tty_incum > 0) ? tp->tty_incum : EAGAIN;
		tp->tty_inleft = 0;
		tp->tty_incum = 0;
		tp->tty_incaller = NONE;
		return r;
	}

	if (tp->tty_select_ops)
		select_retry(tp);

	return EDONTREPLY;
}

static ssize_t do_write(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp;
	ssize_t ret;

	tp = line2tty(minor);
	if (tp == NULL)
		return ENXIO;

	if (tp->tty_outcaller != NONE || tp->tty_outleft > 0)
		return EIO;

	if (size == 0)
		return EINVAL;

	tp->tty_outcaller = endpt;
	tp->tty_outid = id;
	tp->tty_outgrant = grant;
	assert(tp->tty_outcum == 0);
	tp->tty_outleft = size;

	handle_events(tp);

	if (tp->tty_outleft == 0)
		return EDONTREPLY;

	if (flags & CDEV_NONBLOCK) {
		ret = (tp->tty_outcum > 0) ? (ssize_t)tp->tty_outcum : EAGAIN;
		tp->tty_outleft = 0;
		tp->tty_outcum = 0;
		tp->tty_outcaller = NONE;
		return ret;
	}

	if (tp->tty_select_ops)
		select_retry(tp);

	return EDONTREPLY;
}

static int handle_termios_get(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	if (tp == NULL) {
		return EINVAL;
	}

	const vir_bytes src = (vir_bytes)&tp->tty_termios;
	const size_t len = sizeof(tp->tty_termios);

	return sys_safecopyto(endpt, grant, 0, src, len);
}

static int handle_termios_set(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp, unsigned long request)
{
	int r;

	if (tp == NULL)
		return EINVAL;

	if (request == TIOCSETAF) {
		tty_icancel(tp);
	}

	r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&tp->tty_termios, (size_t)sizeof(tp->tty_termios));
	if (r != OK) {
		return r;
	}

	setattr(tp);
	return OK;
}

static int handle_flush(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	int r;
	int flags;

	if (tp == NULL) {
		return EINVAL;
	}

	r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&flags, sizeof(flags));
	if (r != OK) {
		return r;
	}

	if (flags & FREAD) {
		tty_icancel(tp);
	}
	if ((flags & FWRITE) && tp->tty_ocancel != NULL) {
		tp->tty_ocancel(tp, 0);
	}

	return OK;
}

static int handle_winsize(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp,
                         unsigned long request)
{
	if (tp == NULL) return EINVAL;

	const vir_bytes winsz_addr = (vir_bytes)&tp->tty_winsize;
	const size_t winsz_len = sizeof(tp->tty_winsize);

	if (request == TIOCGWINSZ) {
		return sys_safecopyto(endpt, grant, 0, winsz_addr, winsz_len);
	} else {
		int r = sys_safecopyfrom(endpt, grant, 0, winsz_addr, winsz_len);
		if (r == OK) sigchar(tp, SIGWINCH, 0);
		return r;
	}
}

static int handle_bell(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	kio_bell_t bell;
	uint64_t ticks64 = 0;
	clock_t ticks;
	int r;

	if (!isconsole(tp))
		return OK;

	r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&bell, sizeof(bell));
	if (r != OK)
		return r;

	{
		long sec = bell.kb_duration.tv_sec;
		long usec = bell.kb_duration.tv_usec;

		if (sec < 0) sec = 0;
		if (usec < 0) usec = 0;
		if (usec >= 1000000L) {
			sec += usec / 1000000L;
			usec %= 1000000L;
		}

		ticks64 = (uint64_t)sec * (uint64_t)system_hz;
		ticks64 += ((uint64_t)usec * (uint64_t)system_hz) / 1000000ULL;

		if (ticks64 == 0)
			ticks64 = 1;

		{
			unsigned int bits = sizeof(clock_t) * 8;
			uint64_t max_ticks;

			if ((clock_t)-1 < 0) {
				if (bits >= 64) max_ticks = 0x7FFFFFFFFFFFFFFFULL;
				else max_ticks = (1ULL << (bits - 1)) - 1ULL;
			} else {
				if (bits >= 64) max_ticks = 0xFFFFFFFFFFFFFFFFULL;
				else max_ticks = (1ULL << bits) - 1ULL;
			}

			if (ticks64 > max_ticks) ticks = (clock_t)max_ticks;
			else ticks = (clock_t)ticks64;
		}
	}

	beep_x(bell.kb_pitch, ticks);
	return OK;
}

static int copyout_int(endpoint_t endpt, cp_grant_id_t grant, int value)
{
	return sys_safecopyto(endpt, grant, 0, (vir_bytes)&value, sizeof(value));
}

static int handle_output_pending(tty_t *tp, unsigned long request, endpoint_t endpt,
	cp_grant_id_t grant, int flags, cdev_id_t id)
{
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
	return OK;
}

static int do_ioctl(devminor_t minor, unsigned long request, endpoint_t endpt,
	cp_grant_id_t grant, int flags, endpoint_t user_endpt, cdev_id_t id)
{
	tty_t *tp;

	tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	switch (request) {
	case TIOCGETA:
		return handle_termios_get(endpt, grant, tp);

	case TIOCSETAW:
	case TIOCSETAF: {
		int rr = handle_output_pending(tp, request, endpt, grant, flags, id);
		if (rr != OK) return rr;
		return handle_termios_set(endpt, grant, tp, request);
	}

	case TIOCDRAIN: {
		int rr = handle_output_pending(tp, request, endpt, grant, flags, id);
		if (rr != OK) return rr;
		return OK;
	}

	case TIOCSETA:
		return handle_termios_set(endpt, grant, tp, request);

	case TIOCFLUSH:
		return handle_flush(endpt, grant, tp);

	case TIOCSTART:
		tp->tty_inhibited = 0;
		tp->tty_events = 1;
		return OK;

	case TIOCSTOP:
		tp->tty_inhibited = 1;
		tp->tty_events = 1;
		return OK;

	case TIOCSBRK:
		if (tp->tty_break_on) tp->tty_break_on(tp, 0);
		return OK;

	case TIOCCBRK:
		if (tp->tty_break_off) tp->tty_break_off(tp, 0);
		return OK;

	case TIOCGWINSZ:
	case TIOCSWINSZ:
		return handle_winsize(endpt, grant, tp, request);

	case KIOCBELL:
		return handle_bell(endpt, grant, tp);

	case TIOCGETD:
		return copyout_int(endpt, grant, TTYDISC);

	case TIOCSETD:
		printf("TTY: TIOCSETD: can't set any other line discipline.\n");
		return ENOTTY;

	case TIOCGLINED:
		return sys_safecopyto(endpt, grant, 0, (vir_bytes)lined, sizeof(lined));

	case TIOCGQSIZE:
		return copyout_int(endpt, grant, TTY_IN_BYTES);

	case KIOCSMAP:
		if (isconsole(tp)) return kbd_loadmap(endpt, grant);
		return OK;

	case TIOCSFON:
		if (isconsole(tp)) return con_loadfont(endpt, grant);
		return OK;

	case TIOCSCTTY:
		tp->tty_pgrp = user_endpt;
		return OK;

	case TIOCGPGRP:
	case TIOCSPGRP:
	default:
		return ENOTTY;
	}
}

static int do_open(devminor_t minor, int access, endpoint_t user_endpt)
{
	tty_t *tp = line2tty(minor);
	int r = OK;

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
		r = CDEV_CTTY;
	}

	tp->tty_openct++;
	if (tp->tty_openct == 1) {
		tp->tty_open(tp, 0);
	}

	return r;
}

static int do_close(devminor_t minor)
{
	tty_t *tp;
	int affect_openct;

	tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	affect_openct = (minor != LOG_MINOR) || !isconsole(tp);

	if (affect_openct) {
		tp->tty_openct--;
		if (tp->tty_openct == 0) {
			tp->tty_pgrp = 0;
			tty_icancel(tp);
			if (tp->tty_ocancel) {
				tp->tty_ocancel(tp, 0);
			}
			if (tp->tty_close) {
				tp->tty_close(tp, 0);
			}
			tp->tty_termios = termios_defaults;
			tp->tty_winsize = winsize_defaults;
			setattr(tp);
		}
	}

	return OK;
}

static int do_cancel(devminor_t minor, endpoint_t endpt, cdev_id_t id)
{
	tty_t *tp = line2tty(minor);

	if (tp == NULL)
		return ENXIO;

	if (tp->tty_inleft != 0 && endpt == tp->tty_incaller && id == tp->tty_inid) {
		int res;

		tty_icancel(tp);
		res = (tp->tty_incum > 0) ? tp->tty_incum : EAGAIN;

		tp->tty_inleft = 0;
		tp->tty_incum = 0;
		tp->tty_incaller = NONE;
		tp->tty_events = 1;

		return res;
	}

	if (tp->tty_outleft != 0 && endpt == tp->tty_outcaller && id == tp->tty_outid) {
		int res = (tp->tty_outcum > 0) ? tp->tty_outcum : EAGAIN;

		tp->tty_outleft = 0;
		tp->tty_outcum = 0;
		tp->tty_outcaller = NONE;
		tp->tty_events = 1;

		return res;
	}

	if (tp->tty_ioreq != 0 && endpt == tp->tty_iocaller && id == tp->tty_ioid) {
		tp->tty_ioreq = 0;
		tp->tty_iocaller = NONE;
		tp->tty_events = 1;

		return EINTR;
	}

	return EDONTREPLY;
}

int select_try(struct tty *tp, int ops)
{
    int ready_ops = 0;

    if (tp == NULL) {
        return 0;
    }

    if (tp->tty_termios.c_ospeed == B0) {
        return ops;
    }

    if ((ops & CDEV_OP_RD) &&
        (tp->tty_inleft > 0 ||
         (tp->tty_incount > 0 &&
          (!(tp->tty_termios.c_lflag & ICANON) || tp->tty_eotct > 0)))) {
        ready_ops |= CDEV_OP_RD;
    }

    if ((ops & CDEV_OP_WR) &&
        (tp->tty_outleft > 0 ||
         (tp->tty_devwrite != NULL && (*tp->tty_devwrite)(tp, 1)))) {
        ready_ops |= CDEV_OP_WR;
    }

    return ready_ops;
}

int select_retry(struct tty *tp)
{
	int ops;
	int pending_ops;

	if (tp == NULL) {
		return OK;
	}

	pending_ops = tp->tty_select_ops;
	if (pending_ops == 0) {
		return OK;
	}

	ops = select_try(tp, pending_ops);
	if (ops != 0) {
		chardriver_reply_select(tp->tty_select_proc, tp->tty_select_minor, ops);
		tp->tty_select_ops &= ~ops;
	}
	return OK;
}

static int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt)
{
	tty_t *tp = line2tty(minor);
	const int watch = (ops & CDEV_NOTIFY) != 0;
	const unsigned int sel_ops = ops & (CDEV_OP_RD | CDEV_OP_WR | CDEV_OP_ERR);
	const unsigned int ready_ops = (unsigned int)select_try(tp, sel_ops);
	const unsigned int pending_ops = sel_ops & ~ready_ops;

	if (tp == NULL)
		return ENXIO;

	if (pending_ops != 0 && watch) {
		if (tp->tty_select_ops != 0 && tp->tty_select_minor != minor) {
			printf("TTY: select on one object with two minors (%d, %d)\n",
				tp->tty_select_minor, minor);
			return EBADF;
		}
		tp->tty_select_ops |= pending_ops;
		tp->tty_select_proc = endpt;
		tp->tty_select_minor = minor;
	}

	return (int)ready_ops;
}

void handle_events(tty_t *tp)
{
    if (tp == NULL) {
        return;
    }

    for (;;) {
        tp->tty_events = 0;

        if (tp->tty_devread != NULL) {
            tp->tty_devread(tp, 0);
        }

        if (tp->tty_devwrite != NULL) {
            tp->tty_devwrite(tp, 0);
        }

        if (tp->tty_ioreq != 0) {
            dev_ioctl(tp);
        }

        if (tp->tty_events == 0) {
            break;
        }
    }

    in_transfer(tp);
}