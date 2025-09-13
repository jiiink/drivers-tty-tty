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
		if (tp != NULL && tp->tty_events) {
			handle_events(tp);
		}
	}
}

static void handle_kernel_notification(message *tty_mess, int ipc_status)
{
	if (!is_ipc_notify(ipc_status) || tty_mess == NULL) {
		return;
	}
		
	int source_endpoint = _ENDPOINT_P(tty_mess->m_source);
	
	if (source_endpoint == CLOCK) {
		expire_timers(tty_mess->m_notify.timestamp);
		return;
	}
	
	if (source_endpoint == HARDWARE) {
#if NR_RS_LINES > 0
		if (tty_mess->m_notify.interrupts & rs_irq_set) {
			rs_interrupt(tty_mess);
		}
#endif
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
}

int main(void)
{
	message tty_mess;
	int ipc_status;
	int line;
	int r;

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
	int len;

	if (tp == NULL) {
		return;
	}

	buf[0] = '\033';
	len = snprintf(&buf[1], sizeof(buf) - 1, "[1;%dm", color);
	
	if (len < 0 || len >= (int)(sizeof(buf) - 1)) {
		return;
	}

	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) buf, 
		len + 1, CDEV_NONBLOCK, 0);
}

static void reset_color(tty_t *tp)
{
	const char reset_sequence[] = "\033[0;39m";
	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) reset_sequence, 
		sizeof(reset_sequence) - 1, CDEV_NONBLOCK, 0);
}

tty_t *line2tty(devminor_t line)
{
    devminor_t adjusted_line = line;
    
    if (adjusted_line == CONS_MINOR || adjusted_line == LOG_MINOR) {
        adjusted_line = consoleline;
    }
    
    if (adjusted_line == VIDEO_MINOR) {
        return NULL;
    }
    
    tty_t* tp = NULL;
    devminor_t offset;
    
    if (adjusted_line >= CONS_MINOR) {
        offset = adjusted_line - CONS_MINOR;
        if (offset < NR_CONS) {
            tp = tty_addr(offset);
        }
    }
    
    if (tp == NULL && adjusted_line >= RS232_MINOR) {
        offset = adjusted_line - RS232_MINOR;
        if (offset < NR_RS_LINES) {
            tp = tty_addr(offset + NR_CONS);
        }
    }
    
    if (tp != NULL && !tty_active(tp)) {
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

static void set_console_line(char term[CONS_ARG])
{
    const size_t term_len = CONS_ARG - 1;
    
    if (!strncmp(term, "console", term_len)) {
        consoleline = CONS_MINOR;
        return;
    }
    
    for (int i = 1; i < NR_CONS; i++) {
        char cons[6] = "ttyc0";
        cons[4] = '0' + i;
        size_t cmp_len = term_len < sizeof(cons) - 1 ? term_len : sizeof(cons) - 1;
        if (!strncmp(term, cons, cmp_len)) {
            consoleline = CONS_MINOR + i;
            return;
        }
    }
    
    if (NR_RS_LINES > 9) {
        return;
    }
    
    for (int i = 0; i < NR_RS_LINES; i++) {
        char sercons[6] = "tty00";
        sercons[4] = '0' + i;
        size_t cmp_len = term_len < sizeof(sercons) - 1 ? term_len : sizeof(sercons) - 1;
        if (!strncmp(term, sercons, cmp_len)) {
            consoleline = RS232_MINOR + i;
            return;
        }
    }
}

static void set_kernel_color(char color[CONS_ARG])
{
	int def_color;
	int sgr_color;

#define SGR_COLOR_START	30
#define SGR_COLOR_END	37

	def_color = atoi(color);
	sgr_color = SGR_COLOR_START + def_color;
	
	if (sgr_color >= SGR_COLOR_START && sgr_color <= SGR_COLOR_END) {
		kernel_msg_color = sgr_color;
	}
}

static void copy_kernel_messages(char *kernel_buf_copy, struct kmessages *kmess_ptr, 
                                 int prev_next, int next, int bytes)
{
    if (kernel_buf_copy == NULL || kmess_ptr == NULL) {
        return;
    }
    
    if (bytes <= 0 || prev_next < 0 || prev_next >= _KMESS_BUF_SIZE) {
        return;
    }
    
    int first_part_size = MIN(_KMESS_BUF_SIZE - prev_next, bytes);
    if (first_part_size > 0) {
        memcpy(kernel_buf_copy, &kmess_ptr->km_buf[prev_next], first_part_size);
    }
    
    int remaining_bytes = bytes - first_part_size;
    if (remaining_bytes > 0 && remaining_bytes <= _KMESS_BUF_SIZE) {
        memcpy(&kernel_buf_copy[first_part_size], &kmess_ptr->km_buf[0], remaining_bytes);
    }
}

static void do_new_kmess(void)
{
	struct kmessages *kmess_ptr;
	char kernel_buf_copy[_KMESS_BUF_SIZE];
	static int prev_next = 0;
	int next, bytes;
	tty_t *tp, rtp;
	int restore = 0;

	kmess_ptr = get_minix_kerninfo()->kmessages;
	if (kmess_ptr == NULL) {
		return;
	}

	next = kmess_ptr->km_next;
	bytes = ((next + _KMESS_BUF_SIZE) - prev_next) % _KMESS_BUF_SIZE;
	
	if (bytes <= 0) {
		prev_next = next;
		return;
	}

	copy_kernel_messages(kernel_buf_copy, kmess_ptr, prev_next, next, bytes);

	tp = line2tty(consoleline);
	if (tp == NULL) {
		panic("Don't know where to send kernel messages");
	}
		
	if (tp->tty_outleft > 0) {
		rtp = *tp;
		tp->tty_outleft = 0;
		restore = 1;
	}

	if (kernel_msg_color != 0) {
		set_color(tp, kernel_msg_color);
	}
		
	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) kernel_buf_copy, 
	         bytes, CDEV_NONBLOCK, 0);
	         
	if (kernel_msg_color != 0) {
		reset_color(tp);
	}
		
	if (restore != 0) {
		*tp = rtp;
	}

	prev_next = next;
}

static void sef_cb_signal_handler(int signo)
{
	if (signo == SIGKMESS) {
		do_new_kmess();
	} else if (signo == SIGTERM) {
		cons_stop();
	}
}

static ssize_t do_read(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp;

	tp = line2tty(minor);
	if (tp == NULL)
		return ENXIO;

	if (tp->tty_incaller != NONE || tp->tty_inleft > 0)
		return EIO;

	if (size == 0)
		return EINVAL;

	tp->tty_incaller = endpt;
	tp->tty_inid = id;
	tp->tty_ingrant = grant;
	assert(tp->tty_incum == 0);
	tp->tty_inleft = size;

	handle_vtime_settings(tp);
	in_transfer(tp);
	handle_events(tp);
	
	if (tp->tty_inleft == 0)
		return EDONTREPLY;

	if (flags & CDEV_NONBLOCK)
		return handle_nonblock_read(tp);

	if (tp->tty_select_ops)
		select_retry(tp);

	return EDONTREPLY;
}

static void handle_vtime_settings(tty_t *tp)
{
	if ((tp->tty_termios.c_lflag & ICANON) || tp->tty_termios.c_cc[VTIME] == 0)
		return;

	if (tp->tty_termios.c_cc[VMIN] == 0) {
		settimer(tp, TRUE);
		tp->tty_min = 1;
	} else if (tp->tty_eotct == 0) {
		settimer(tp, FALSE);
		tp->tty_min = tp->tty_termios.c_cc[VMIN];
	}
}

static ssize_t handle_nonblock_read(tty_t *tp)
{
	ssize_t result;

	tty_icancel(tp);
	result = tp->tty_incum > 0 ? tp->tty_incum : EAGAIN;
	tp->tty_inleft = 0;
	tp->tty_incum = 0;
	tp->tty_incaller = NONE;
	return result;
}

static ssize_t do_write(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp;
	ssize_t result;

	tp = line2tty(minor);
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
	tp->tty_outcum = 0;
	tp->tty_outleft = size;

	handle_events(tp);
	
	if (tp->tty_outleft == 0) {
		return EDONTREPLY;
	}

	if ((flags & CDEV_NONBLOCK) != 0) {
		result = (tp->tty_outcum > 0) ? tp->tty_outcum : EAGAIN;
		tp->tty_outleft = 0;
		tp->tty_outcum = 0;
		tp->tty_outcaller = NONE;
		return result;
	}

	if (tp->tty_select_ops != 0) {
		select_retry(tp);
	}

	return EDONTREPLY;
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
	int r;
	
	if (tp == NULL) {
		return EINVAL;
	}
	
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
	int flush_flags;
	int r;

	if (tp == NULL) {
		return EINVAL;
	}

	r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&flush_flags, sizeof(flush_flags));
	if (r != OK) {
		return r;
	}
		
	if (flush_flags & FREAD) {
		tty_icancel(tp);
	}
	
	if ((flush_flags & FWRITE) && (tp->tty_ocancel != NULL)) {
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
	if (r != OK) {
		return r;
	}
	
	sigchar(tp, SIGWINCH, 0);
	return OK;
}

static int handle_bell(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	kio_bell_t bell;
	clock_t ticks;
	int r;
	
	if (!isconsole(tp))
		return OK;
		
	r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &bell, sizeof(bell));
	if (r != OK)
		return r;
		
	ticks = bell.kb_duration.tv_usec * system_hz / 1000000;
	ticks += bell.kb_duration.tv_sec * system_hz;
	if (ticks == 0)
		ticks = 1;
		
	beep_x(bell.kb_pitch, ticks);
	return OK;
}

static int do_ioctl(devminor_t minor, unsigned long request, endpoint_t endpt,
	cp_grant_id_t grant, int flags, endpoint_t user_endpt, cdev_id_t id)
{
	tty_t *tp;
	int i, r;

	tp = line2tty(minor);
	if (tp == NULL)
		return ENXIO;

	r = OK;
	switch (request) {
	case TIOCGETA:
		r = handle_termios_get(endpt, grant, tp);
		break;

	case TIOCSETAW:
	case TIOCSETAF:
	case TIOCDRAIN:
		if (tp->tty_outleft > 0) {
			if (flags & CDEV_NONBLOCK)
				return EAGAIN;
			tp->tty_iocaller = endpt;
			tp->tty_ioid = id;
			tp->tty_ioreq = request;
			tp->tty_iogrant = grant;
			return EDONTREPLY;
		}
		if (request == TIOCDRAIN) 
			break;
		r = handle_termios_set(endpt, grant, tp, request);
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
		if (tp->tty_break_on != NULL) 
			(*tp->tty_break_on)(tp, 0);
		break;
		
	case TIOCCBRK:
		if (tp->tty_break_off != NULL) 
			(*tp->tty_break_off)(tp, 0);
		break;

	case TIOCGWINSZ:
	case TIOCSWINSZ:
		r = handle_winsize(endpt, grant, tp, request);
		break;
		
	case KIOCBELL:
		r = handle_bell(endpt, grant, tp);
		break;
		
	case TIOCGETD:
		i = TTYDISC;
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &i, sizeof(i));
		break;
		
	case TIOCSETD:
		printf("TTY: TIOCSETD: can't set any other line discipline.\n");
		r = ENOTTY;
		break;
		
	case TIOCGLINED:
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes) lined, sizeof(lined));
		break;
		
	case TIOCGQSIZE:
		i = TTY_IN_BYTES;
		r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &i, sizeof(i));
		break;
		
	case KIOCSMAP:
		if (isconsole(tp)) 
			r = kbd_loadmap(endpt, grant);
		else
			r = ENOTTY;
		break;

	case TIOCSFON:
		if (isconsole(tp)) 
			r = con_loadfont(endpt, grant);
		else
			r = ENOTTY;
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

	tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	if (minor == LOG_MINOR && isconsole(tp)) {
		if ((access & CDEV_R_BIT) != 0) {
			return EACCES;
		}
		return OK;
	}

	if ((access & CDEV_NOCTTY) == 0) {
		tp->tty_pgrp = user_endpt;
	}

	tp->tty_openct++;
	if (tp->tty_openct == 1 && tp->tty_open != NULL) {
		(*tp->tty_open)(tp, 0);
	}

	return ((access & CDEV_NOCTTY) == 0) ? CDEV_CTTY : OK;
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

	if (tp->tty_openct > 0) {
		tp->tty_openct--;
	}

	if (tp->tty_openct == 0) {
		tp->tty_pgrp = 0;
		tty_icancel(tp);
		
		if (tp->tty_ocancel != NULL) {
			(*tp->tty_ocancel)(tp, 0);
		}
		
		if (tp->tty_close != NULL) {
			(*tp->tty_close)(tp, 0);
		}
		
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

	tp = line2tty(minor);
	if (tp == NULL) {
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
		return ops;
	}

	if ((ops & CDEV_OP_RD) != 0) {
		if (tp->tty_inleft > 0) {
			ready_ops |= CDEV_OP_RD;
		} else if (tp->tty_incount > 0) {
			int canon_disabled = (tp->tty_termios.c_lflag & ICANON) == 0;
			int has_eot = tp->tty_eotct > 0;
			
			if (canon_disabled || has_eot) {
				ready_ops |= CDEV_OP_RD;
			}
		}
	}

	if ((ops & CDEV_OP_WR) != 0) {
		if (tp->tty_outleft > 0) {
			ready_ops |= CDEV_OP_WR;
		} else if (tp->tty_devwrite != NULL && (*tp->tty_devwrite)(tp, 1) != 0) {
			ready_ops |= CDEV_OP_WR;
		}
	}
	
	return ready_ops;
}

int select_retry(struct tty *tp)
{
	int ops;

	if (tp->tty_select_ops == 0) {
		return OK;
	}

	ops = select_try(tp, tp->tty_select_ops);
	if (ops == 0) {
		return OK;
	}

	chardriver_reply_select(tp->tty_select_proc, tp->tty_select_minor, ops);
	tp->tty_select_ops &= ~ops;

	return OK;
}

static int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt)
{
	tty_t *tp;
	int ready_ops;
	int watch;
	unsigned int filtered_ops;

	tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}

	watch = (ops & CDEV_NOTIFY);
	filtered_ops = ops & (CDEV_OP_RD | CDEV_OP_WR | CDEV_OP_ERR);

	ready_ops = select_try(tp, filtered_ops);
	filtered_ops &= ~ready_ops;
	
	if (filtered_ops != 0 && watch != 0) {
		if (tp->tty_select_ops != 0 && tp->tty_select_minor != minor) {
			printf("TTY: select on one object with two minors (%d, %d)\n",
				tp->tty_select_minor, minor);
			return EBADF;
		}
		tp->tty_select_ops |= filtered_ops;
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

	if (tp->tty_incum >= tp->tty_min && tp->tty_inleft > 0) {
		tty_reply(tp->tty_inrepcode, tp->tty_incaller, tp->tty_inproc, tp->tty_incum);
		tp->tty_inleft = 0;
	}
}