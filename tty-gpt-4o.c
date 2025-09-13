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

#include <stdbool.h>

static void process_tty_events(void)
{
    for (tty_t *tp = FIRST_TTY; tp < END_TTY; tp++) {
        if (tp->tty_events) {
            handle_events(tp);
        }
    }
}

#include <stdbool.h>

static void handle_kernel_notification(message *tty_mess, int ipc_status) {
    if (!is_ipc_notify(ipc_status)) return;

    int source_endpoint = _ENDPOINT_P(tty_mess->m_source);
    bool handle_clock = (source_endpoint == CLOCK);
    bool handle_hardware = (source_endpoint == HARDWARE);

    if (handle_clock || handle_hardware) {
        if (handle_hardware) {
#if NR_RS_LINES > 0
            if (tty_mess->m_notify.interrupts & rs_irq_set) {
                rs_interrupt(tty_mess);
            }
#endif
        }
        expire_timers(tty_mess->m_notify.timestamp);
    }
}

static int handle_special_message(message *tty_mess, int ipc_status, int line) {
    if (tty_mess == NULL) {
        return -1; // Error: null message
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

    if (IS_CDEV_RQ(tty_mess->m_type) == 0) {
        chardriver_process(&tty_tab, tty_mess, ipc_status);
        return 1;
    }

    if (line == VIDEO_MINOR) {
        do_video(tty_mess, ipc_status);
        return 1;
    }

    return 0;
}

#include <stdbool.h>

int main(void) {
    message tty_mess;
    int ipc_status;
    int line;
    int r;

    sef_local_startup();

    while (true) {
        process_tty_events();

        r = driver_receive(ANY, &tty_mess, &ipc_status);
        if (r != 0) {
            log_error("driver_receive failed", r);
            continue;
        }

        if (is_ipc_notify(ipc_status)) {
            handle_kernel_notification(&tty_mess, ipc_status);
            continue;
        }

        if (chardriver_get_minor(&tty_mess, &line) != OK) {
            log_debug("Invalid minor number");
            continue;
        }

        if (handle_special_message(&tty_mess, ipc_status, line)) {
            log_debug("Special message handled");
            continue;
        }

        if (chardriver_process(&tty_tab, &tty_mess, ipc_status) != OK) {
            log_error("chardriver_process failed", 0);
        }
    }

    return 0;
}

#include <stdio.h>
#include <string.h>

static void set_color(tty_t *tp, int color) {
    char buf[8];
    int buf_len;

    buf[0] = '\033';
    buf_len = snprintf(&buf[1], sizeof(buf) - 1, "[1;%dm", color);
    if (buf_len < 0 || (size_t)buf_len >= sizeof(buf) - 1) {
        // handle error, perhaps with logging
        return;
    }

    do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)buf, strlen(buf), CDEV_NONBLOCK, 0);
}

static void reset_color(tty_t *tp) {
    char buf[8];
    const char sgrColorReset = 39;
    
    buf[0] = '\033';
    snprintf(&buf[1], sizeof(buf) - 1, "[0;%dm", sgrColorReset);
    
    if (do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)buf, sizeof(buf), CDEV_NONBLOCK, 0) < 0) {
        // Handle error: could add logging or some other mechanism
    }
}

tty_t *line2tty(devminor_t line) {
    tty_t *tp = NULL;
    devminor_t adjusted_line = line;

    if (line == CONS_MINOR || line == LOG_MINOR) {
        adjusted_line = consoleline;
    }

    if (adjusted_line != VIDEO_MINOR) {
        if ((adjusted_line - CONS_MINOR) < NR_CONS) {
            tp = tty_addr(adjusted_line - CONS_MINOR);
        } else if ((adjusted_line - RS232_MINOR) < NR_RS_LINES) {
            tp = tty_addr(adjusted_line - RS232_MINOR + NR_CONS);
        }

        if (tp != NULL && !tty_active(tp)) {
            tp = NULL;
        }
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

#include <string.h>
#include <errno.h>

#define CHECK_SYS_CALL(call) do { if ((call) != OK) return -1; } while(0)
#define CHECK_ENV_PARAM(param, buffer) do { if (env_get_param((param), (buffer), sizeof(buffer)) == OK) process_env_param((buffer)); } while(0)

static int retrieve_machine_info(void) {
    int result;
    result = sys_getmachine(&machine);
    if (result != OK) {
        return result;
    }
    return OK;
}

static void process_env_param(const char* param) {
    if (strcmp(param, "console") == 0) {
        set_console_line(param);
    } else if (strcmp(param, "kernelclr") == 0) {
        set_kernel_color(param);
    }
}

static int sef_cb_init_fresh(int UNUSED(type), sef_init_info_t *UNUSED(info)) {
    int result;
    char value[CONS_ARG];

    result = retrieve_machine_info();
    if (result != OK) {
        panic("Couldn't obtain kernel environment: %d", result);
        return result;
    }

    CHECK_ENV_PARAM("console", value);
    CHECK_ENV_PARAM("kernelclr", value);

    tty_init();
    kb_init_once();
    sys_diagctl_register();

    return OK;
}

#include <string.h>
#include <assert.h>

static void set_console_line(char term[CONS_ARG]) {
    int i;
    char consBase[] = "ttyc0";
    char serconsBase[] = "tty00";

    if (strncmp(term, "console", CONS_ARG - 1) == 0) {
        consoleline = CONS_MINOR + 0;
        return;
    }

    for (i = 1; i < NR_CONS; i++) {
        consBase[4] = '0' + i;
        if (strncmp(term, consBase, strnlen(term, CONS_ARG)) == 0) {
            consoleline = CONS_MINOR + i;
            return;
        }
    }

    assert(NR_RS_LINES <= 9);
    for (i = 0; i < NR_RS_LINES; i++) {
        serconsBase[4] = '0' + i;
        if (strncmp(term, serconsBase, strnlen(term, CONS_ARG)) == 0) {
            consoleline = RS232_MINOR + i;
            return;
        }
    }
}

#include <stdlib.h>

static void set_kernel_color(const char *color) {
    int def_color;
    int color_code;

    def_color = atoi(color);
    color_code = SGR_COLOR_START + def_color;

    if (color_code >= SGR_COLOR_START && color_code <= SGR_COLOR_END) {
        kernel_msg_color = color_code;
    }
}

static void copy_kernel_messages(char *kernel_buf_copy, const struct kmessages *kmess_ptr, int prev_next, int next, int bytes) {
    int first_copy_len = MIN(_KMESS_BUF_SIZE - prev_next, bytes);
    int second_copy_len = bytes - first_copy_len;
    
    memcpy(kernel_buf_copy, &kmess_ptr->km_buf[prev_next], first_copy_len);
    
    if (second_copy_len > 0) {
        memcpy(&kernel_buf_copy[first_copy_len], kmess_ptr->km_buf, second_copy_len);
    }
}

static void do_new_kmess(void) {
    struct kmessages *kmess_ptr = get_minix_kerninfo()->kmessages;
    int next = kmess_ptr->km_next;
    int bytes = ((next + _KMESS_BUF_SIZE) - prev_next) % _KMESS_BUF_SIZE;

    if (bytes > 0) {
        char kernel_buf_copy[_KMESS_BUF_SIZE];
        tty_t *tp = line2tty(consoleline);

        if (tp == NULL) {
            panic("Don't know where to send kernel messages");
            return;
        }

        copy_kernel_messages(kernel_buf_copy, kmess_ptr, prev_next, next, bytes);

        tty_t rtp = *tp;
        bool restore = tp->tty_outleft > 0;
        if (restore) {
            tp->tty_outleft = 0;
        }

        if (kernel_msg_color != 0) {
            set_color(tp, kernel_msg_color);
        }

        do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t)kernel_buf_copy, bytes, CDEV_NONBLOCK, 0);

        if (kernel_msg_color != 0) {
            reset_color(tp);
        }

        if (restore) {
            *tp = rtp;
        }
    }

    prev_next = next;
}

void sef_cb_signal_handler(int signo) {
    if (signo == SIGKMESS) {
        do_new_kmess();
    } else if (signo == SIGTERM) {
        cons_stop();
    } else {
        // Handle unexpected signal
    }
}

static ssize_t do_read(devminor_t minor, u64_t UNUSED(position),
    endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
    cdev_id_t id)
{
    tty_t *tp = line2tty(minor);

    if (tp == NULL) return ENXIO;

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
        } else if (tp->tty_eotct == 0) {
            settimer(tp, FALSE);
            tp->tty_min = tp->tty_termios.c_cc[VMIN];
        }
    }

    in_transfer(tp);
    handle_events(tp);

    if (tp->tty_inleft == 0) return EDONTREPLY;

    if (flags & CDEV_NONBLOCK) {
        tty_icancel(tp);
        int read_status = tp->tty_incum > 0 ? tp->tty_incum : EAGAIN;
        tp->tty_inleft = tp->tty_incum = 0;
        tp->tty_incaller = NONE;
        return read_status;
    }

    if (tp->tty_select_ops) select_retry(tp);

    return EDONTREPLY;
}

static ssize_t do_write(devminor_t minor, u64_t position, endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags, cdev_id_t id) {
    tty_t *tp = line2tty(minor);
    if (tp == NULL) return ENXIO;

    if (tp->tty_outcaller != NONE || tp->tty_outleft > 0 || size <= 0) return EIO;

    tp->tty_outcaller = endpt;
    tp->tty_outid = id;
    tp->tty_outgrant = grant;
    tp->tty_outcum = 0;
    tp->tty_outleft = size;

    handle_events(tp);

    if (tp->tty_outleft == 0) return EDONTREPLY;

    if (flags & CDEV_NONBLOCK) {
        int r = tp->tty_outcum > 0 ? tp->tty_outcum : EAGAIN;
        tp->tty_outleft = tp->tty_outcum = 0;
        tp->tty_outcaller = NONE;
        return r;
    }

    if (tp->tty_select_ops) select_retry(tp);

    return EDONTREPLY;
}

#include <errno.h>

static int handle_termios_get(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp) {
	if (!tp) {
		return EINVAL;
	}

	int result = sys_safecopyto(endpt, grant, 0, (vir_bytes)&tp->tty_termios, sizeof(struct termios));
	if (result != OK) {
		return result;
	}

	return OK;
}

static int handle_termios_set(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp, unsigned long request) {
	if (request == TIOCSETAF) {
		tty_icancel(tp);
	}

	int r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &tp->tty_termios, sizeof(struct termios));
	if (r != OK) {
		return r;
	}
	
	setattr(tp);
	return OK;
}

static int handle_flush(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp) {
	int i;
	int r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &i, sizeof(i));
	if (r != OK) {
		return r;
	}
		
	if (i & FREAD) {
		tty_icancel(tp);
	}
	if (i & FWRITE) {
		tp->tty_ocancel(tp, 0);
	}
		
	return OK;
}

#include <errno.h>

static int handle_winsize(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp, unsigned long request) {
    int result;

    if (request == TIOCGWINSZ) {
        result = sys_safecopyto(endpt, grant, 0, (vir_bytes)&tp->tty_winsize, sizeof(struct winsize));
        if (result != OK) {
            return result;
        }
    } else {
        result = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&tp->tty_winsize, sizeof(struct winsize));
        if (result != OK) {
            return result;
        }
        sigchar(tp, SIGWINCH, 0);
    }

    return OK;
}

static int handle_bell(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp) {
    kio_bell_t bell;
    clock_t ticks;

    if (!isconsole(tp)) {
        return OK;
    }

    int r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes)&bell, sizeof(bell));
    if (r != OK) {
        return r;
    }

    ticks = (clock_t)((bell.kb_duration.tv_usec / 1000000.0 + bell.kb_duration.tv_sec) * system_hz);
    if (ticks == 0) {
        ticks = 1;
    }

    beep_x(bell.kb_pitch, ticks);
    return OK;
}

#include <errno.h>

static int do_ioctl(devminor_t minor, unsigned long request, endpoint_t endpt,
    cp_grant_id_t grant, int flags, endpoint_t user_endpt, cdev_id_t id)
{
    tty_t *tp = line2tty(minor);
    if (!tp) return ENXIO;

    int r = OK;

    switch (request) {
        case TIOCGETA:
            return handle_termios_get(endpt, grant, tp);

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
            if (request == TIOCDRAIN) break;

        case TIOCSETA:
            return handle_termios_set(endpt, grant, tp, request);

        case TIOCFLUSH:
            return handle_flush(endpt, grant, tp);

        case TIOCSTART:
            tp->tty_inhibited = 0;
            tp->tty_events = 1;
            break;

        case TIOCSTOP:
            tp->tty_inhibited = 1;
            tp->tty_events = 1;
            break;

        case TIOCSBRK:
            if (tp->tty_break_on) tp->tty_break_on(tp, 0);
            break;

        case TIOCCBRK:
            if (tp->tty_break_off) tp->tty_break_off(tp, 0);
            break;

        case TIOCGWINSZ:
        case TIOCSWINSZ:
            return handle_winsize(endpt, grant, tp, request);

        case KIOCBELL:
            return handle_bell(endpt, grant, tp);

        case TIOCGETD: {
            int i = TTYDISC;
            return sys_safecopyto(endpt, grant, 0, (vir_bytes) &i, sizeof(i));
        }

        case TIOCSETD:
            printf("TTY: TIOCSETD: can't set any other line discipline.\n");
            return ENOTTY;

        case TIOCGLINED:
            return sys_safecopyto(endpt, grant, 0, (vir_bytes) lined, sizeof(lined));

        case TIOCGQSIZE: {
            int i = TTY_IN_BYTES;
            return sys_safecopyto(endpt, grant, 0, (vir_bytes) &i, sizeof(i));
        }

        case KIOCSMAP:
            if (isconsole(tp)) return kbd_loadmap(endpt, grant);
            break;

        case TIOCSFON:
            if (isconsole(tp)) return con_loadfont(endpt, grant);
            break;

        case TIOCSCTTY:
            tp->tty_pgrp = user_endpt;
            break;

        default:
            return ENOTTY;
    }

    return r;
}

int do_open(devminor_t minor, int access, endpoint_t user_endpt) {
	tty_t *tp = line2tty(minor);
	if (tp == NULL) {
		return ENXIO;
	}
	
	if (minor == LOG_MINOR && isconsole(tp)) {
		return (access & CDEV_R_BIT) ? EACCES : OK;
	}

	if (!(access & CDEV_NOCTTY)) {
		tp->tty_pgrp = user_endpt;
	}

	tp->tty_openct++;
	if (tp->tty_openct == 1) {
		tp->tty_open(tp, 0);
	}
	
	return !(access & CDEV_NOCTTY) ? CDEV_CTTY : OK;
}

static int do_close(devminor_t minor) {
    tty_t *tp = line2tty(minor);
    if (tp == NULL) return ENXIO;

    if (minor == LOG_MINOR && isconsole(tp)) return OK;

    if (--tp->tty_openct == 0) {
        tp->tty_pgrp = 0;
        tty_icancel(tp);
        tp->tty_ocancel(tp, 0);
        tp->tty_close(tp, 0);
        tp->tty_termios = termios_defaults;
        tp->tty_winsize = winsize_defaults;
        setattr(tp);
    }

    return OK;
}

static int do_cancel(devminor_t minor, endpoint_t endpt, cdev_id_t id)
{
    tty_t *tp = line2tty(minor);
    if (tp == NULL) return ENXIO;

    int r = EDONTREPLY;
    if (endpt == tp->tty_incaller && id == tp->tty_inid) {
        if (tp->tty_inleft != 0) {
            tty_icancel(tp);
            r = (tp->tty_incum > 0) ? tp->tty_incum : EAGAIN;
            tp->tty_inleft = tp->tty_incum = 0;
            tp->tty_incaller = NONE;
        }
    } else if (endpt == tp->tty_outcaller && id == tp->tty_outid) {
        if (tp->tty_outleft != 0) {
            r = (tp->tty_outcum > 0) ? tp->tty_outcum : EAGAIN;
            tp->tty_outleft = tp->tty_outcum = 0;
            tp->tty_outcaller = NONE;
        }
    } else if (endpt == tp->tty_iocaller && id == tp->tty_ioid) {
        if (tp->tty_ioreq != 0) {
            r = EINTR;
            tp->tty_ioreq = 0;
            tp->tty_iocaller = NONE;
        }
    }

    if (r != EDONTREPLY) {
        tp->tty_events = 1;
    }

    return r;
}

int select_try(struct tty *tp, int ops) {
    int ready_ops = 0;

    if (tp->tty_termios.c_ospeed == B0) {
        return ops;
    }

    if ((ops & CDEV_OP_RD) && (tp->tty_inleft > 0 || 
        (tp->tty_incount > 0 && (!(tp->tty_termios.c_lflag & ICANON) || tp->tty_eotct > 0)))) {
        ready_ops |= CDEV_OP_RD;
    }

    if ((ops & CDEV_OP_WR) && (tp->tty_outleft > 0 || (*tp->tty_devwrite)(tp, 1))) {
        ready_ops |= CDEV_OP_WR;
    }

    return ready_ops;
}

int select_retry(struct tty *tp) {
	if (tp->tty_select_ops) {
		int ops = select_try(tp, tp->tty_select_ops);
		if (ops) {
			chardriver_reply_select(tp->tty_select_proc, tp->tty_select_minor, ops);
			tp->tty_select_ops &= ~ops;
		}
	}
	return OK;
}

int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt) {
	tty_t *tp = line2tty(minor);
	if (!tp) return ENXIO;

	unsigned int clean_ops = ops & (CDEV_OP_RD | CDEV_OP_WR | CDEV_OP_ERR);
	int ready_ops = select_try(tp, clean_ops);

	if (clean_ops &= ~ready_ops) {
		if ((ops & CDEV_NOTIFY) && (tp->tty_select_ops == 0 || tp->tty_select_minor == minor)) {
			tp->tty_select_ops |= clean_ops;
			tp->tty_select_proc = endpt;
			tp->tty_select_minor = minor;
		} else if (tp->tty_select_ops != 0 && tp->tty_select_minor != minor) {
			printf("TTY: select on one object with two minors (%d, %d)\n", tp->tty_select_minor, minor);
			return EBADF;
		}
	}

	return ready_ops;
}

void handle_events(tty_t *tp) {
    if (tp == NULL || tp->tty_devread == NULL || tp->tty_devwrite == NULL) {
        return;
    }
    
    while (1) {
        tp->tty_events = 0;
        tp->tty_devread(tp, 0);
        tp->tty_devwrite(tp, 0);
        
        if (tp->tty_ioreq != 0) {
            dev_ioctl(tp);
        }
        
        if (!tp->tty_events) {
            break;
        }
    }
    
    in_transfer(tp);
    
    if (tp->tty_incum >= tp->tty_min) {
        process_input(tp);
    }
}