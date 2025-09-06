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

#define tty_addr(line) (&tty_table[line])
#define isconsole(tp) ((tp) < tty_addr(NR_CONS))
#define FIRST_TTY tty_addr(0)
#define END_TTY tty_addr(sizeof(tty_table) / sizeof(tty_table[0]))
#define tty_active(tp) ((tp)->tty_devread != NULL)

#if NR_RS_LINES == 0
#define rs_init(tp) ((void)0)
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
static void set_console_line(const char term[CONS_ARG]);
static void set_kernel_color(const char color[CONS_ARG]);
static void write_escape_code(tty_t *tp, const char *format, int value);
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
	.cdr_open = do_open,
	.cdr_close = do_close,
	.cdr_read = do_read,
	.cdr_write = do_write,
	.cdr_ioctl = do_ioctl,
	.cdr_cancel = do_cancel,
	.cdr_select = do_select};

static struct termios termios_defaults = {
	.c_iflag = TTYDEF_IFLAG,
	.c_oflag = TTYDEF_OFLAG,
	.c_cflag = TTYDEF_CFLAG,
	.c_lflag = TTYDEF_LFLAG,
	.c_ispeed = TTYDEF_SPEED,
	.c_ospeed = TTYDEF_SPEED,
	.c_cc = {[VEOF] = CEOF, [VEOL] = CEOL, [VERASE] = CERASE, [VINTR] = CINTR, [VKILL] = CKILL, [VMIN] = CMIN, [VQUIT] = CQUIT, [VTIME] = CTIME, [VSUSP] = CSUSP, [VSTART] = CSTART, [VSTOP] = CSTOP, [VREPRINT] = CREPRINT, [VLNEXT] = CLNEXT, [VDISCARD] = CDISCARD, [VSTATUS] = CSTATUS}};
static struct winsize winsize_defaults;

tty_t tty_table[NR_CONS + NR_RS_LINES];
int ccurrent;
struct machine machine;
u32_t system_hz;
u32_t consoleline = CONS_MINOR;
u32_t kernel_msg_color = 0;

static const char lined[TTLINEDNAMELEN] = "termios";

static void sef_local_startup(void);
static int sef_cb_init_fresh(int type, sef_init_info_t *info);
static void sef_cb_signal_handler(int signo);

int main(void)
{
	message tty_mess;
	int ipc_status;
	devminor_t line;
	int r;
	tty_t *tp;

	sef_local_startup();
	while (TRUE) {
		for (tp = FIRST_TTY; tp < END_TTY; tp++) {
			if (tp->tty_events) handle_events(tp);
		}

		if ((r = driver_receive(ANY, &tty_mess, &ipc_status)) != 0) {
			panic("driver_receive failed with: %d", r);
		}

		if (is_ipc_notify(ipc_status)) {
			switch (_ENDPOINT_P(tty_mess.m_source)) {
			case CLOCK:
				expire_timers(tty_mess.m_notify.timestamp);
				break;
			case HARDWARE:
#if NR_RS_LINES > 0
				if (tty_mess.m_notify.interrupts & rs_irq_set)
					rs_interrupt(&tty_mess);
#endif
				expire_timers(tty_mess.m_notify.timestamp);
				break;
			default:
				break;
			}
		} else {
			switch (tty_mess.m_type) {
			case TTY_FKEY_CONTROL:
				do_fkey_ctl(&tty_mess);
				break;
			case TTY_INPUT_UP:
			case TTY_INPUT_EVENT:
				do_input(&tty_mess);
				break;
			default:
				if (IS_CDEV_RQ(tty_mess.m_type)) {
					if (chardriver_get_minor(&tty_mess, &line) == OK) {
						if (line == VIDEO_MINOR) {
							do_video(&tty_mess, ipc_status);
						} else {
							chardriver_process(&tty_tab, &tty_mess, ipc_status);
						}
					}
				} else {
					chardriver_process(&tty_tab, &tty_mess, ipc_status);
				}
				break;
			}
		}
	}

	return 0;
}

static void write_escape_code(tty_t *tp, const char *format, int value)
{
	char buf[12];
	int len;

	buf[0] = '\033';
	len = snprintf(&buf[1], sizeof(buf) - 1, format, value);

	if (len > 0 && (size_t)len < sizeof(buf) - 1) {
		do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)buf, (size_t)1 + len,
			CDEV_NONBLOCK, 0);
	}
}

static void set_color(tty_t *tp, int color)
{
	write_escape_code(tp, "[1;%dm", color);
}

static void reset_color(tty_t *tp)
{
#define SGR_COLOR_RESET 39
	write_escape_code(tp, "[0;%dm", SGR_COLOR_RESET);
}

tty_t *line2tty(devminor_t line)
{
	tty_t *tp = NULL;

	if (line == CONS_MINOR || line == LOG_MINOR) {
		line = consoleline;
	}

	if (line == VIDEO_MINOR) {
		return NULL;
	} else if ((line - CONS_MINOR) < NR_CONS) {
		tp = tty_addr(line - CONS_MINOR);
	} else if ((line - RS232_MINOR) < NR_RS_LINES) {
		tp = tty_addr(line - RS232_MINOR + NR_CONS);
	}

	if (tp != NULL && !tty_active(tp)) {
		tp = NULL;
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

static int sef_cb_init_fresh(int UNUSED(type), sef_init_info_t *UNUSED(info))
{
	int r;
	char val[CONS_ARG];

	if ((r = sys_getmachine(&machine)) != OK) {
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

static void set_console_line(const char term[CONS_ARG])
{
	int i;

	if (strncmp(term, "console", sizeof("console") - 1) == 0) {
		consoleline = CONS_MINOR + 0;
		return;
	}

	for (i = 1; i < NR_CONS; i++) {
		char cons[6];
		strlcpy(cons, "ttyc0", sizeof(cons));
		cons[4] += i;
		if (strncmp(term, cons, sizeof(cons) - 1) == 0) {
			consoleline = CONS_MINOR + i;
			return;
		}
	}

	assert(NR_RS_LINES <= 9);
	for (i = 0; i < NR_RS_LINES; i++) {
		char sercons[6];
		strlcpy(sercons, "tty00", sizeof(sercons));
		sercons[4] += i;
		if (strncmp(term, sercons, sizeof(sercons) - 1) == 0) {
			consoleline = RS232_MINOR + i;
			return;
		}
	}
}

static void set_kernel_color(const char color[CONS_ARG])
{
#define SGR_COLOR_START 30
#define SGR_COLOR_END 37
#define SGR_COLOR_COUNT (SGR_COLOR_END - SGR_COLOR_START + 1)
	int def_color = atoi(color);
	if (def_color >= 0 && def_color < SGR_COLOR_COUNT) {
		kernel_msg_color = (u32_t)def_color + SGR_COLOR_START;
	}
}

static int get_new_kmess_bytes(char *buffer, size_t buf_size)
{
	static int prev_next = 0;
	struct kmessages *kmess_ptr = get_minix_kerninfo()->kmessages;
	int next = kmess_ptr->km_next;
	int bytes = ((next + _KMESS_BUF_SIZE) - prev_next) % _KMESS_BUF_SIZE;

	if (bytes <= 0 || (size_t)bytes > buf_size) {
		prev_next = next;
		return 0;
	}

	int copy = MIN(_KMESS_BUF_SIZE - prev_next, bytes);
	memcpy(buffer, &kmess_ptr->km_buf[prev_next], copy);

	if (copy < bytes) {
		memcpy(&buffer[copy], &kmess_ptr->km_buf[0], (size_t)bytes - copy);
	}

	prev_next = next;
	return bytes;
}

static void print_kernel_message(tty_t *tp, const char *buffer, size_t size)
{
	tty_t rtp;
	int restore = 0;

	if (tp->tty_outleft > 0) {
		rtp = *tp;
		tp->tty_outleft = 0;
		restore = 1;
	}

	if (kernel_msg_color != 0) {
		set_color(tp, kernel_msg_color);
	}

	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)buffer, size, CDEV_NONBLOCK, 0);

	if (kernel_msg_color != 0) {
		reset_color(tp);
	}

	if (restore) {
		*tp = rtp;
	}
}

static void do_new_kmess(void)
{
	char kernel_buf_copy[_KMESS_BUF_SIZE];
	int bytes = get_new_kmess_bytes(kernel_buf_copy, sizeof(kernel_buf_copy));

	if (bytes > 0) {
		tty_t *tp = line2tty(consoleline);
		if (tp == NULL) {
			panic("Don't know where to send kernel messages");
		}
		print_kernel_message(tp, kernel_buf_copy, bytes);
	}
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
	}
}

static ssize_t do_read(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp;
	int r;

	if ((tp = line2tty(minor)) == NULL) return ENXIO;
	if (tp->tty_incaller != NONE || tp->tty_inleft > 0) return EIO;
	if (size <= 0) return EINVAL;

	tp->tty_incaller = endpt;
	tp->tty_inid = id;
	tp->tty_ingrant = grant;
	assert(tp->tty_incum == 0);
	tp->tty_inleft = size;

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
	if (tp->tty_inleft == 0) return EDONTREPLY;

	if (flags & CDEV_NONBLOCK) {
		tty_icancel(tp);
		r = (tp->tty_incum > 0) ? (int)tp->tty_incum : EAGAIN;
		tp->tty_inleft = tp->tty_incum = 0;
		tp->tty_incaller = NONE;
		return r;
	}

	if (tp->tty_select_ops) select_retry(tp);

	return EDONTREPLY;
}

static ssize_t do_write(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp;
	int r;

	if ((tp = line2tty(minor)) == NULL) return ENXIO;
	if (tp->tty_outcaller != NONE || tp->tty_outleft > 0) return EIO;
	if (size <= 0) return EINVAL;

	tp->tty_outcaller = endpt;
	tp->tty_outid = id;
	tp->tty_outgrant = grant;
	assert(tp->tty_outcum == 0);
	tp->tty_outleft = size;

	handle_events(tp);
	if (tp->tty_outleft == 0) return EDONTREPLY;

	if (flags & CDEV_NONBLOCK) {
		r = (tp->tty_outcum > 0) ? (int)tp->tty_outcum : EAGAIN;
		tp->tty_outleft = tp->tty_outcum = 0;
		tp->tty_outcaller = NONE;
		return r;
	}

	if (tp->tty_select_ops) select_retry(tp);

	return EDONTREPLY;
}

static int do_ioctl_seta_now(tty_t *tp, endpoint_t endpt, cp_grant_id_t grant)
{
	int r;

	r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&tp->tty_termios,
		sizeof(struct termios));
	if (r != OK) {
		return r;
	}
	setattr(tp);
	return OK;
}

static int do_ioctl(devminor_t minor, unsigned long request, endpoint_t endpt,
	cp_grant_id_t grant, int flags, endpoint_t user_endpt, cdev_id_t id)
{
	kio_bell_t bell;
	clock_t ticks;
	tty_t *tp;
	int i, r;

	if ((tp = line2tty(minor)) == NULL) return ENXIO;

	r = OK;
	switch (request) {
	case TIOCGETA:
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes)&tp->tty_termios,
			sizeof(struct termios));
		break;
	case TIOCSETA:
		r = do_ioctl_seta_now(tp, endpt, grant);
		break;
	case TIOCSETAW:
	case TIOCSETAF:
	case TIOCDRAIN:
		if (tp->tty_outleft > 0) {
			if (flags & CDEV_NONBLOCK) {
				r = EAGAIN;
				break;
			}
			tp->tty_iocaller = endpt;
			tp->tty_ioid = id;
			tp->tty_ioreq = request;
			tp->tty_iogrant = grant;
			r = EDONTREPLY;
			break;
		}
		if (request == TIOCDRAIN) break;
		if (request == TIOCSETAF) tty_icancel(tp);
		r = do_ioctl_seta_now(tp, endpt, grant);
		break;
	case TIOCFLUSH:
		r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&i, sizeof(i));
		if (r != OK) break;
		if (i & FREAD) tty_icancel(tp);
		if (i & FWRITE) (*tp->tty_ocancel)(tp, 0);
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
		if (tp->tty_break_on != NULL) (*tp->tty_break_on)(tp, 0);
		break;
	case TIOCCBRK:
		if (tp->tty_break_off != NULL) (*tp->tty_break_off)(tp, 0);
		break;
	case TIOCGWINSZ:
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes)&tp->tty_winsize,
			sizeof(struct winsize));
		break;
	case TIOCSWINSZ:
		r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&tp->tty_winsize,
			sizeof(struct winsize));
		sigchar(tp, SIGWINCH, 0);
		break;
	case KIOCBELL:
		if (!isconsole(tp)) break;
		r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&bell, sizeof(bell));
		if (r != OK) break;
		ticks = bell.kb_duration.tv_usec * system_hz / 1000000;
		ticks += bell.kb_duration.tv_sec * system_hz;
		if (!ticks) ticks++;
		beep_x(bell.kb_pitch, ticks);
		break;
	case TIOCGETD:
		i = TTYDISC;
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes)&i, sizeof(i));
		break;
	case TIOCSETD:
		r = ENOTTY;
		break;
	case TIOCGLINED:
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes)lined, sizeof(lined));
		break;
	case TIOCGQSIZE:
		i = TTY_IN_BYTES;
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes)&i, sizeof(i));
		break;
	case KIOCSMAP:
		if (isconsole(tp)) r = kbd_loadmap(endpt, grant);
		break;
	case TIOCSFON:
		if (isconsole(tp)) r = con_loadfont(endpt, grant);
		break;
	case TIOCSCTTY:
		tp->tty_pgrp = user_endpt;
		break;
	case TIOCGPGRP:
	case TIOCSPGRP:
	default:
		r = ENOTTY;
	}
	return r;
}

static int do_open(devminor_t minor, int access, endpoint_t user_endpt)
{
	tty_t *tp;
	int r = OK;

	if ((tp = line2tty(minor)) == NULL) return ENXIO;

	if (minor == LOG_MINOR && isconsole(tp)) {
		if (access & CDEV_R_BIT) return EACCES;
	} else {
		if (!(access & CDEV_NOCTTY)) {
			tp->tty_pgrp = user_endpt;
			r = CDEV_CTTY;
		}
		tp->tty_openct++;
		if (tp->tty_openct == 1) {
			(*tp->tty_open)(tp, 0);
		}
	}

	return r;
}

static int do_close(devminor_t minor)
{
	tty_t *tp;

	if ((tp = line2tty(minor)) == NULL) return ENXIO;

	if ((minor != LOG_MINOR || !isconsole(tp)) && --tp->tty_openct == 0) {
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

	if ((tp = line2tty(minor)) == NULL) return ENXIO;

	r = EDONTREPLY;
	if (tp->tty_inleft != 0 && endpt == tp->tty_incaller && id == tp->tty_inid) {
		tty_icancel(tp);
		r = (tp->tty_incum > 0) ? (int)tp->tty_incum : EAGAIN;
		tp->tty_inleft = tp->tty_incum = 0;
		tp->tty_incaller = NONE;
	} else if (tp->tty_outleft != 0 && endpt == tp->tty_outcaller &&
		id == tp->tty_outid) {
		r = (tp->tty_outcum > 0) ? (int)tp->tty_outcum : EAGAIN;
		tp->tty_outleft = tp->tty_outcum = 0;
		tp->tty_outcaller = NONE;
	} else if (tp->tty_ioreq != 0 && endpt == tp->tty_iocaller &&
		id == tp->tty_ioid) {
		r = EINTR;
		tp->tty_ioreq = 0;
		tp->tty_iocaller = NONE;
	}
	if (r != EDONTREPLY) tp->tty_events = 1;
	return r;
}

int select_try(struct tty *tp, int ops)
{
	int ready_ops = 0;

	if (tp->tty_termios.c_ospeed == B0) {
		ready_ops |= ops;
	}

	if (ops & CDEV_OP_RD) {
		if (tp->tty_inleft > 0) {
			ready_ops |= CDEV_OP_RD;
		} else if (tp->tty_incount > 0) {
			if (!(tp->tty_termios.c_lflag & ICANON) ||
				tp->tty_eotct > 0) {
				ready_ops |= CDEV_OP_RD;
			}
		}
	}

	if (ops & CDEV_OP_WR) {
		if (tp->tty_outleft > 0) {
			ready_ops |= CDEV_OP_WR;
		} else if ((*tp->tty_devwrite)(tp, 1)) {
			ready_ops |= CDEV_OP_WR;
		}
	}
	return ready_ops;
}

int select_retry(struct tty *tp)
{
	int ops;

	if (tp->tty_select_ops && (ops = select_try(tp, tp->tty_select_ops))) {
		chardriver_reply_select(tp->tty_select_proc,
			tp->tty_select_minor, ops);
		tp->tty_select_ops &= ~ops;
	}
	return OK;
}

static int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt)
{
	tty_t *tp;
	int ready_ops, watch;

	if ((tp = line2tty(minor)) == NULL) return ENXIO;

	watch = (ops & CDEV_NOTIFY);
	ops &= (CDEV_OP_RD | CDEV_OP_WR | CDEV_OP_ERR);

	ready_ops = select_try(tp, ops);

	ops &= ~ready_ops;
	if (ops && watch) {
		if (tp->tty_select_ops != 0 && tp->tty_select_minor != minor) {
			return EBADF;
		}
		tp->tty_select_ops |= ops;
		tp->tty_select_proc = endpt;
		tp->tty_select_minor = minor;
	}

	return ready_ops;
}

void handle_events(tty_t *tp)
{
	do {
		tp->tty_events = 0;
		(*tp->tty_devread)(tp, 0);
		(*tp->tty_devwrite)(tp, 0);
		if (tp->tty_ioreq != 0) dev_ioctl(tp);
	} while (tp->tty_events);

	in_transfer(tp);

	if (tp->tty_incum >= tp->tty_min && tp->tty_inleft > 0) {
		chardriver_reply_task(tp->tty_incaller, tp->tty_inid, tp->tty_incum);
		tp->tty_inleft = tp->tty_incum = 0;
		tp->tty_incaller = NONE;
	}
	if (tp->tty_select_ops) {
		select_retry(tp);
	}
}

static void in_transfer(tty_t *tp)
{
	int ch;
	int count;
	char buf[64], *bp;

	if (tp->tty_termios.c_ospeed == B0) tp->tty_min = 0;

	if (tp->tty_inleft == 0 || tp->tty_eotct < tp->tty_min) return;

	bp = buf;
	while (tp->tty_inleft > 0 && tp->tty_eotct > 0) {
		ch = *tp->tty_intail;

		if (!(ch & IN_EOF)) {
			*bp = (char)(ch & IN_CHAR);
			tp->tty_inleft--;
			if (++bp == bufend(buf)) {
				sys_safecopyto(tp->tty_incaller,
					tp->tty_ingrant, tp->tty_incum,
					(vir_bytes)buf, (vir_bytes)buflen(buf));
				tp->tty_incum += buflen(buf);
				bp = buf;
			}
		}

		if (++tp->tty_intail == bufend(tp->tty_inbuf)) {
			tp->tty_intail = tp->tty_inbuf;
		}
		tp->tty_incount--;
		if (ch & IN_EOT) {
			tp->tty_eotct--;
			if (tp->tty_termios.c_lflag & ICANON) tp->tty_inleft = 0;
		}
	}

	if (bp > buf) {
		count = bp - buf;
		sys_safecopyto(tp->tty_incaller, tp->tty_ingrant, tp->tty_incum,
			(vir_bytes)buf, (vir_bytes)count);
		tp->tty_incum += count;
	}

	if (tp->tty_inleft == 0) {
		chardriver_reply_task(tp->tty_incaller, tp->tty_inid, tp->tty_incum);
		tp->tty_inleft = tp->tty_incum = 0;
		tp->tty_incaller = NONE;
	}
}

int in_process(tty_t *tp, const char *buf, int count);
static void tty_timed_out(int arg);
static void settimer(tty_t *tty_ptr, int enable);
static int tty_echo(tty_t *tp, int ch);
static void rawecho(tty_t *tp, int ch);
static int back_over(tty_t *tp);
static void reprint(tty_t *tp);
static void dev_ioctl(tty_t *tp);
static void setattr(tty_t *tp);
static void tty_icancel(tty_t *tp);
static void tty_init(void);