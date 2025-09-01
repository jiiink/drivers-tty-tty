```c
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

#define MAX_BUF_SIZE 64
#define IN_EOF 0x100
#define IN_ESC 0x200
#define IN_EOT 0x400
#define IN_CHAR 0xff
#define IN_LEN 0xff00
#define IN_LSHIFT 8

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
static void set_console_line(char term[CONS_ARG]);
static void set_kernel_color(char color[CONS_ARG]);
static void set_color(tty_t *tp, int color);
static void reset_color(tty_t *tp);

static int do_open(devminor_t minor, int access, endpoint_t user_endpt);
static int do_close(devminor_t minor);
static ssize_t do_read(devminor_t minor, u64_t position, endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags, cdev_id_t id);
static ssize_t do_write(devminor_t minor, u64_t position, endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags, cdev_id_t id);
static int do_ioctl(devminor_t minor, unsigned long request, endpoint_t endpt, cp_grant_id_t grant, int flags, endpoint_t user_endpt, cdev_id_t id);
static int do_cancel(devminor_t minor, endpoint_t endpt, cdev_id_t id);
static int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt);

static struct chardriver tty_tab = {
    .cdr_open   = do_open,
    .cdr_close  = do_close,
    .cdr_read   = do_read,
    .cdr_write  = do_write,
    .cdr_ioctl  = do_ioctl,
    .cdr_cancel = do_cancel,
    .cdr_select = do_select
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

int main(void) {
    message tty_mess;
    int ipc_status;
    int line;
    int r;
    register tty_t *tp;

    sef_local_startup();

    while (TRUE) {
        for (tp = FIRST_TTY; tp < END_TTY; tp++) {
            if (tp->tty_events) handle_events(tp);
        }

        r = driver_receive(ANY, &tty_mess, &ipc_status);
        if (r != 0)
            panic("driver_receive failed with: %d", r);

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
            continue;
        }

        switch (tty_mess.m_type) {
            case TTY_FKEY_CONTROL:
                do_fkey_ctl(&tty_mess);
                continue;
            case TTY_INPUT_UP:
            case TTY_INPUT_EVENT:
                do_input(&tty_mess);
                continue;
            default:
                break;
        }

        if (!IS_CDEV_RQ(tty_mess.m_type)){
            chardriver_process(&tty_tab, &tty_mess, ipc_status);
            continue;
        }

        if (OK != chardriver_get_minor(&tty_mess, &line))
            continue;

        if (line == VIDEO_MINOR) {
            do_video(&tty_mess, ipc_status);
            continue;
        }

        chardriver_process(&tty_tab, &tty_mess, ipc_status);
    }
    return 0;
}

static void set_color(tty_t *tp, int color) {
    char buf[8];
    buf[0] = '\033';
    snprintf(&buf[1], sizeof(buf) - 1, "[1;%dm", color);
    do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) buf, sizeof(buf), CDEV_NONBLOCK, 0);
}

static void reset_color(tty_t *tp) {
    char buf[8];
    #define SGR_COLOR_RESET 39
    buf[0] = '\033';
    snprintf(&buf[1], sizeof(buf) - 1, "[0;%dm", SGR_COLOR_RESET);
    do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) buf, sizeof(buf), CDEV_NONBLOCK, 0);
}

tty_t *line2tty(devminor_t line) {
    tty_t *tp;

    if (line == CONS_MINOR || line == LOG_MINOR)
        line = consoleline;

    if (line == VIDEO_MINOR) {
        return(NULL);
    } else if ((line - CONS_MINOR) < NR_CONS) {
        tp = tty_addr(line - CONS_MINOR);
    } else if ((line - RS232_MINOR) < NR_RS_LINES) {
        tp = tty_addr(line - RS232_MINOR + NR_CONS);
    } else {
        tp = NULL;
    }

    if (tp != NULL && !tty_active(tp))
        tp = NULL;

    return(tp);
}

static void sef_local_startup() {
    sef_setcb_init_fresh(sef_cb_init_fresh);
    sef_setcb_init_restart(SEF_CB_INIT_RESTART_STATEFUL);
    sef_setcb_signal_handler(sef_cb_signal_handler);
    sef_startup();
}

static int sef_cb_init_fresh(int UNUSED(type), sef_init_info_t *UNUSED(info)) {
    int r;
    char val[CONS_ARG];

    if (OK != (r=sys_getmachine(&machine))) {
        panic("Couldn't obtain kernel environment: %d", r);
    }

    if (env_get_param("console", val, sizeof(val)) == OK) {
        set_console_line(val);
    }

    if ((r = env_get_param("kernelclr", val, sizeof(val))) == OK) {
        set_kernel_color(val);
    }

    tty_init();
    kb_init_once();
    sys_diagctl_register();
    return(OK);
}

static void set_console_line(char term[CONS_ARG]) {
    int i;

    if (!strncmp(term, "console", CONS_ARG - 1)) {
        consoleline = CONS_MINOR + 0;
    }

    for (i = 1; i < NR_CONS; i++) {
        char cons[6];
        strlcpy(cons, "ttyc0", sizeof(cons));
        cons[4] += i;
        if (!strncmp(term, cons, CONS_ARG < sizeof(cons) ? CONS_ARG-1 : sizeof(cons) - 1))
            consoleline = CONS_MINOR + i;
    }

    assert(NR_RS_LINES <= 9);
    for (i = 0; i < NR_RS_LINES; i++) {
        char sercons[6];
        strlcpy(sercons, "tty00", sizeof(sercons));
        sercons[4] += i;
        if (!strncmp(term, sercons, CONS_ARG < sizeof(sercons) ? CONS_ARG-1 : sizeof(sercons)-1))
            consoleline = RS232_MINOR + i;
    }
}

static void set_kernel_color(char color[CONS_ARG]) {
    int def_color;
    #define SGR_COLOR_START 30
    #define SGR_COLOR_END 37
    def_color = atoi(color);
    if ((SGR_COLOR_START + def_color) >= SGR_COLOR_START && (SGR_COLOR_START + def_color) <= SGR_COLOR_END) {
        kernel_msg_color = def_color + SGR_COLOR_START;
    }
}

static void do_new_kmess(void) {
    struct kmessages *kmess_ptr;
    char kernel_buf_copy[_KMESS_BUF_SIZE];
    static int prev_next = 0;
    int next, bytes, copy, restore = 0;
    tty_t *tp, rtp;

    kmess_ptr = get_minix_kerninfo()->kmessages;
    next = kmess_ptr->km_next;
    bytes = ((next + _KMESS_BUF_SIZE) - prev_next) % _KMESS_BUF_SIZE;
    if (bytes > 0) {
        copy = MIN(_KMESS_BUF_SIZE - prev_next, bytes);
        memcpy(kernel_buf_copy, &kmess_ptr->km_buf[prev_next], copy);

        if (copy < bytes) {
            memcpy(&kernel_buf_copy[copy], &kmess_ptr->km_buf[0], bytes - copy);
        }

        tp = line2tty(consoleline);
        if (tp == NULL)
            panic("Don't know where to send kernel messages");
        if (tp->tty_outleft > 0) {
            rtp = *tp;
            tp->tty_outleft = 0;
            restore = 1;
        }

        if (kernel_msg_color != 0)
            set_color(tp, kernel_msg_color);
        do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) kernel_buf_copy, bytes, CDEV_NONBLOCK, 0);
        if (kernel_msg_color != 0)
            reset_color(tp);
        if (restore) {
            *tp = rtp;
        }
    }

    prev_next = next;
}

static void sef_cb_signal_handler(int signo) {
    switch(signo) {
        case SIGKMESS:
            do_new_kmess();
            break;
        case SIGTERM:
            cons_stop();
            break;
    }
}

static ssize_t do_read(devminor_t minor, u64_t UNUSED(position), endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags, cdev_id_t id) {
    tty_t *tp;

    if ((tp = line2tty(minor)) == NULL)
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
    if (tp->tty_inleft == 0)
        return EDONTREPLY;

    if (flags & CDEV_NONBLOCK) {
        tty_icancel(tp);
        int r = tp->tty_incum > 0 ? tp->tty_incum : EAGAIN;
        tp->tty_inleft = tp->tty_incum = 0;
        tp->tty_incaller = NONE;
        return r;
    }

    if (tp->tty_select_ops)
        select_retry(tp);

    return EDONTREPLY;
}

static ssize_t do_write(devminor_t minor, u64_t UNUSED(position), endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags, cdev_id_t id) {
    tty_t *tp;

    if ((tp = line2tty(minor)) == NULL)
        return ENXIO;

    if (tp->tty_outcaller != NONE || tp->tty_outleft > 0)
        return EIO;
    if (size <= 0)
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
        int r = tp->tty_outcum > 0 ? tp->tty_outcum : EAGAIN;
        tp->tty_outleft = tp->tty_outcum = 0;
        tp->tty_outcaller = NONE;
        return r;
    }

    if (tp->tty_select_ops)
        select_retry(tp);

    return EDONTREPLY;
}

static int do_ioctl(devminor_t minor, unsigned long request, endpoint_t endpt, cp_grant_id_t grant, int flags, endpoint_t user_endpt, cdev_id_t id) {
    kio_bell_t bell;
    clock_t ticks;
    tty_t *tp;
    int i, r;

    if ((tp = line2tty(minor)) == NULL)
        return ENXIO;

    r = OK;
    switch (request) {
        case TIOCGETA:
            r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &tp->tty_termios, sizeof(struct termios));
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
            if (request == TIOCDRAIN) break;
            if (request == TIOCSETAF) tty_icancel(tp);
        case TIOCSETA:
            r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &tp->tty_termios, sizeof(struct termios));
            if (r != OK) break;
            setattr(tp);
            break;
        case TIOCFLUSH:
            r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &i, sizeof(i));
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
            r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &tp->tty_winsize, sizeof(struct winsize));
            break;
        case TIOCSWINSZ:
            r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &tp->tty_winsize, sizeof(struct winsize));
            sigchar(tp, SIGWINCH, 0);
            break;
        case KIOCBELL:
            if (!isconsole(tp))
                break;
            r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &bell, sizeof(bell));
            if (r != OK)
                break;
            ticks = bell.kb_duration.tv_usec * system_hz / 1000000;
            ticks += bell.kb_duration.tv_sec * system_hz;
            if (!ticks)
                ticks++;
            beep_x(bell.kb_pitch, ticks);
            break;
        case TIOCGETD:
            i = TTYDISC;
            r = sys_safecopyto(endpt, grant, 0, (vir_bytes) &i, sizeof(i));
            break;
        case TIOCSETD:
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
            if (isconsole(tp)) r = kbd_loadmap(endpt, grant);
            break;
        case TIOCSFON:
            if (isconsole(tp)) r = con_loadfont(endpt, grant);
            break;
        case TIOCSCTTY:
            tp->tty_pgrp = user_endpt;
            break;
        default:
            r = ENOTTY;
    }
    return r;
}

static int do_open(devminor_t minor, int access, endpoint_t user_endpt) {
    tty_t *tp;
    int r = OK;

    if ((tp = line2tty(minor)) == NULL)
        return ENXIO;

    if (minor == LOG_MINOR && isconsole(tp)) {
        if (access & CDEV_R_BIT) return EACCES;
    } else {
        if (!(access & CDEV_NOCTTY)) {
            tp->tty_pgrp = user_endpt;
            r = CDEV_CTTY;
        }
        tp->tty_openct++;
        if (tp->tty_openct == 1)
            (*tp->tty_open)(tp, 0);
    }
    return r;
}

static int do_close(devminor_t minor) {
    tty_t *tp;

    if ((tp = line2tty(minor)) == NULL)
        return ENXIO;

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

static int do_cancel(devminor_t minor, endpoint_t endpt, cdev_id_t id) {
    tty_t *tp;

    if ((tp = line2tty(minor)) == NULL)
        return ENXIO;

    int r = EDONTREPLY;
    if (tp->tty_inleft != 0 && endpt == tp->tty_incaller && id == tp->tty_inid) {
        tty_icancel(tp);
        r = tp->tty_incum > 0 ? tp->tty_incum : EAGAIN;
        tp->tty_inleft = tp->tty_incum = 0;
        tp->tty_incaller = NONE;
    } else if (tp->tty_outleft != 0 && endpt == tp->tty_outcaller && id == tp->tty_outid) {
        r = tp->tty_outcum > 0 ? tp->tty_outcum : EAGAIN;
        tp->tty_outleft = tp->tty_outcum = 0;
        tp->tty_outcaller = NONE;
    } else if (tp->tty_ioreq != 0 && endpt == tp->tty_iocaller && id == tp->tty_ioid) {
        r = EINTR;
        tp->tty_ioreq = 0;
        tp->tty_iocaller = NONE;
    }
    if (r != EDONTREPLY)
        tp->tty_events = 1;
    return r;
}

int select_try(struct tty *tp, int ops) {
    int ready_ops = 0;

    if (tp->tty_termios.c_ospeed == B0) {
        ready_ops |= ops;
    }

    if (ops & CDEV_OP_RD) {
        if (tp->tty_inleft > 0) {
            ready_ops |= CDEV_OP_RD;
        } else if (tp->tty_incount > 0) {
            if (!(tp->tty_termios.c_lflag & ICANON) || tp->tty_eotct > 0) {
                ready_ops |= CDEV_OP_RD;
            }
        }
    }

    if (ops & CDEV_OP_WR) {
        if (tp->tty_outleft > 0)  ready_ops |= CDEV_OP_WR;
        else if ((*tp->tty_devwrite)(tp, 1)) ready_ops |= CDEV_OP_WR;
    }
    return ready_ops;
}

int select_retry(struct tty *tp) {
    int ops;

    if (tp->tty_select_ops && (ops = select_try(tp, tp->tty_select_ops))) {
        chardriver_reply_select(tp->tty_select_proc, tp->tty_select_minor, ops);
        tp->tty_select_ops &= ~ops;
    }
    return OK;
}

static int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt) {
    tty_t *tp;
    int ready_ops, watch;

    if ((tp = line2tty(minor)) == NULL)
        return ENXIO;

    watch = (ops & CDEV_NOTIFY);
    ops &= (CDEV_OP_RD | CDEV_OP_WR | CDEV_OP_ERR);

    ready_ops = select_try(tp, ops);

    ops &= ~ready_ops;
    if (ops && watch) {
        if (tp->tty_select_ops != 0 && tp->tty_select_minor != minor) {
            printf("TTY: select on one object with two minors (%d, %d)\n", tp->tty_select_minor, minor);
            return EBADF;
        }
        tp->tty_select_ops |= ops;
        tp->tty_select_proc = endpt;
        tp->tty_select_minor = minor;
    }
    return ready_ops;
}

void handle_events(tty_t *tp) {
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

static void in_transfer(tty_t *tp) {
    int ch, count;
    char buf[MAX_BUF_SIZE], *bp;

    if (tp->tty_termios.c_ospeed == B0) tp->tty_min = 0;

    if (tp->tty_inleft == 0 || tp->tty_eotct < tp->tty_min) return;

    bp = buf;
    while (tp->tty_inleft > 0 && tp->tty_eotct > 0) {
        ch = *tp->tty_intail;

        if (!(ch & IN_EOF)) {
            *bp = ch & IN_CHAR;
            tp->tty_inleft--;
            if (++bp == bufend(buf)) {
                sys_safecopyto(tp->tty_incaller, tp->tty_ingrant, tp->tty_incum, (vir_bytes) buf, (vir_bytes) buflen(buf));
                tp->tty_incum += buflen(buf);
                bp = buf;
            }
        }

        if (++tp->tty_intail == bufend(tp->tty_inbuf))
            tp->tty_intail = tp->tty_inbuf;
        tp->tty_incount--;
        if (ch & IN_EOT) {
            tp->tty_eotct--;
            if (tp->tty_termios.c_lflag & ICANON) tp->tty_inleft = 0;
        }
    }

    if (bp > buf) {
        count = bp - buf;
        sys_safecopyto(tp->tty_incaller, tp->tty_ingrant, tp->tty_incum, (vir_bytes) buf, (vir_bytes) count);
        tp->tty_incum += count;
    }

    if (tp->tty_inleft == 0) {
        chardriver_reply_task(tp->tty_incaller, tp->tty_inid, tp->tty_incum);
        tp->tty_inleft = tp->tty_incum = 0;
        tp->tty_incaller = NONE;
    }
}

int in_process(tty_t *tp, char *buf, int count) {
    int ch, sig, ct;
    int timeset = FALSE;

    for (ct = 0; ct < count; ct++) {
        ch = *buf++ & BYTE;

        if (tp->tty_termios.c_iflag & ISTRIP) ch &= 0x7F;

        if (tp->tty_termios.c_lflag & IEXTEN) {
            if (tp->tty_escaped) {
                tp->tty_escaped = NOT_ESCAPED;
                ch |= IN_ESC;
            }
            if (ch == tp->tty_termios.c_cc[VLNEXT]) {
                tp->tty_escaped = ESCAPED;
                rawecho(tp, '^');
                rawecho(tp, '\b');
                continue;
            }
            if (ch == tp->tty_termios.c_cc[VREPRINT]) {
                reprint(tp);
                continue;
            }
        }

        if (ch == _POSIX_VDISABLE) ch |= IN_ESC;

        if (ch == '\r') {
            if (tp->tty_termios.c_iflag & IGNCR) continue;
            if (tp->tty_termios.c_iflag & ICRNL) ch = '\n';
        } else if (ch == '\n') {
            if (tp->tty_termios.c_iflag & INLCR) ch = '\r';
        }

        if (tp->tty_termios.c_lflag & ICANON) {
            if (ch == tp->tty_termios.c_cc[VERASE]) {
                back_over(tp);
                if (!(tp->tty_termios.c_lflag & ECHOE)) {
                    tty_echo(tp, ch);
                }
                continue;
            }

            if (ch == tp->tty_termios.c_cc[VKILL]) {
                while (back_over(tp)) {}
                if (!(tp->tty_termios.c_lflag & ECHOE)) {
                    tty_echo(tp, ch);
                    if (tp->tty_termios.c_lflag & ECHOK)
                        rawecho(tp, '\n');
                }
                continue;
            }

            if (ch == tp->tty_termios.c_cc[VEOF]) ch |= IN_EOT | IN_EOF;
            if (ch == '\n') ch |= IN_EOT;
            if (ch == tp->tty_termios.c_cc[VEOL]) ch |= IN_EOT;
        }

        if (tp->tty_termios.c_iflag & IXON) {
            if (ch == tp->tty_termios.c_cc[VSTOP]) {
                tp->tty_inhibited = STOPPED;
                tp->tty_events = 1;
                continue;
            }

            if (tp->tty_inhibited) {
                if (ch == tp->tty_termios.c_cc[VSTART] || (tp->tty_termios.c_iflag & IXANY)) {
                    tp->tty_inhibited = RUNNING;
                    tp->tty_events = 1;
                    if (ch == tp->tty_termios.c_cc[VSTART])
                        continue;
                }
            }
        }

        if (tp->tty_termios.c_lflag & ISIG) {
            int sig = -1;
            if (ch == tp->tty_termios.c_cc[VINTR])
                sig = SIGINT;
            else if (ch == tp->tty_termios.c_cc[VQUIT])
                sig = SIGQUIT;
            else if (ch == tp->tty_termios.c_cc[VSTATUS])
                sig = SIGINFO;

            if (sig >= 0) {
                sigchar(tp, sig, 1);
                tty_echo(tp, ch);
                continue;
            }
        }

        if (tp->tty_incount == buflen(tp->tty_inbuf)) {
            if (tp->tty_termios.c_lflag & ICANON) continue;
            break;
        }

        if (!(tp->tty_termios.c_lflag & ICANON)) {
            ch |= IN_EOT;

            if (!timeset && tp->tty_termios.c_cc[VMIN] > 0 && tp->tty_termios.c_cc[VTIME] > 0) {
                settimer(tp, TRUE);
                timeset = TRUE;
            }
        }

        if (tp->tty_termios.c_lflag & (ECHO | ECHONL)) ch = tty_echo(tp, ch);

        *tp->tty_inhead++ = ch;
        if (tp->tty_inhead == bufend(tp->tty_inbuf))
            tp->tty_inhead = tp->tty_inbuf;
        tp->tty_incount++;
        if (ch & IN_EOT) tp->tty_eotct++;

        if (tp->tty_incount == buflen(tp->tty_inbuf)) in_transfer(tp);
    }
    return ct;
}

static int tty_echo(tty_t *tp, int ch) {
    int len, rp;

    ch &= ~IN_LEN;
    if (!(tp->tty_termios.c_lflag & ECHO)) {
        if ((ch == ('\n' | IN_EOT)) && (tp->tty_termios.c_lflag & (ICANON | ECHONL)) == (ICANON | ECHONL))
            (*tp->tty_echo)(tp, '\n');
        return ch;
    }

    rp = tp->tty_incount == 0 ? FALSE : tp->tty_reprint;

    if ((ch & IN_CHAR) < ' ') {
        switch (ch & (IN_ESC | IN_EOF | IN_EOT | IN_CHAR)) {
            case '\t':
                len = 0;
                do {
                    (*tp->tty_echo)(tp, ' ');
                    len++;
                } while (len < TAB_SIZE && (tp->tty_position & TAB_MASK) != 0);
                break;
            case '\r' | IN_EOT:
            case '\n' | IN_EOT:
                (*tp->tty_echo)(tp, ch & IN_CHAR);
                len = 0;
                break;
            default:
                (*tp->tty_echo)(tp, '^');
                (*tp->tty_echo)(tp, '@' + (ch & IN_CHAR));
                len = 2;
        }
    } else if ((ch & IN_CHAR) == '\177') {
        (*tp->tty_echo)(tp, '^');
        (*tp->tty_echo)(tp, '?');
        len = 2;
    } else {
        (*tp->tty_echo)(tp, ch & IN_CHAR);
        len = 1;
    }
    if (ch & IN_EOF) while (len > 0) {
        (*tp->tty_echo)(tp, '\b');
        len--;
    }

    tp->tty_reprint = rp;
    return ch | (len << IN_LSHIFT);
}

static void rawecho(tty_t *tp, int ch) {
    int rp = tp->tty_reprint;
    if (tp->tty_termios.c_lflag & ECHO) (*tp->tty_echo)(tp, ch);
    tp->tty_reprint = rp;
}

static int back_over(tty_t *tp) {
    u16_t *head;
    int len;

    if (tp->tty_incount == 0) return 0;

    head = tp->tty_inhead;
    if (head == tp->tty_inbuf) head = bufend(tp->tty_inbuf);
    if (*--head & IN_EOT) return 0;

    if (tp->tty_reprint) reprint(tp);

    tp->tty_inhead = head;
    tp->tty_incount--;
    if (tp->tty_termios.c_lflag & ECHOE) {
        len = (*head & IN_LEN) >> IN_LSHIFT;
        while (len > 0) {
            rawecho(tp, '\b');
            rawecho(tp, ' ');
            rawecho(tp, '\b');
            len--;
        }
    }
    return 1;
}

static void reprint(tty_t *tp) {
    int count;
    u16_t *head;

    tp->tty_reprint = FALSE;

    head = tp->tty_inhead;
    count = tp->tty_incount;
    while (count > 0) {
        if (head == tp->tty_inbuf) head = bufend(tp->tty_inbuf);
        if (head[-1] & IN_EOT) break;
        head--;
        count--;
    }
    if (count == tp->tty_incount) return;

    tty_echo(tp, tp->tty_termios.c_cc[VREPRINT] | IN_ESC);
    rawecho(tp, '\r');
    rawecho(tp, '\n');

    do {
        if (head == bufend(tp->tty_inbuf)) head = tp->tty_inbuf;
        *head = tty_echo(tp, *head);
        head++;
        count++;
    } while (count < tp->tty_incount);
}

void out_process(tty_t *tp, char *bstart, char *bpos, char *bend, int *icount, int *ocount) {
    int tablen;
    int ict = *icount;
    int oct = *ocount;
    int pos = tp->tty_position;

    while (ict > 0) {
        switch (*bpos) {
            case '\7':
                break;
            case '\b':
                pos--;
                break;
            case '\r':
                pos = 0;
                break;
            case '\n':
                if ((tp->tty_termios.c_oflag & (OPOST | ONLCR)) == (OPOST | ONLCR)) {
                    if (oct >= 2) {
                        *bpos = '\r';
                        if (++bpos == bend) bpos = bstart;
                        *bpos = '\n';
                        pos = 0;
                        ict--;
                        oct -= 2;
                    }
                    goto out_done;
                }
                break;
            case '\t':
                tablen = TAB_SIZE - (pos & TAB_MASK);

                if ((tp->tty_termios.c_oflag & (OPOST | OXTABS)) == (OPOST | OXTABS)) {
                    if (oct >= tablen) {
                        pos += tablen;
                        ict--;
                        oct -= tablen;
                        do {
                            *bpos = ' ';
                            if (++bpos == bend) bpos = bstart;
                        } while (--tablen != 0);
                    }
                    goto out_done;
                }
                pos += tablen;
                break;
            default:
                pos++;
        }
        if (++bpos == bend) bpos = bstart;
        ict--;
        oct--;
    }
out_done:
    tp->tty_position = pos & TAB_MASK;

    *icount -= ict;
    *ocount -= oct;
}

static void dev_ioctl(tty_t *tp) {
    int result = EINVAL;

    if (tp->tty_outleft > 0) return;

    if (tp->tty_ioreq != TIOCDRAIN) {
        if (tp->tty_ioreq == TIOCSETAF) tty_icancel(tp);
        result = sys_safecopyfrom(tp->tty_iocaller, tp->tty_iogrant, 0, (vir_bytes) &tp->tty_termios, (vir_bytes) sizeof(tp->tty_termios));
        if (result == OK) setattr(tp);
    }
    tp->tty_ioreq = 0;
    chardriver_reply_task(tp->tty_iocaller, tp->tty_ioid, result);
    tp->tty_iocaller = NONE;
}

static void setattr(tty_t *tp) {
    u16_t *inp;
    int count;

    if (!(tp->tty_termios.c_lflag & ICANON)) {
        count = tp->tty_eotct = tp->tty_incount;
        inp = tp->tty_intail;
        while (count > 0) {
            *inp |= IN_EOT;
            if (++inp == bufend(tp->tty_inbuf)) inp = tp->tty_inbuf;
            --count;
        }
    }

    settimer(tp, FALSE);
    if (tp->tty_termios.c_lflag & ICANON) {
        tp->tty_min = 1;
    } else {
        tp->tty_min = tp->tty_termios.c_cc[VMIN];
        if (tp->tty_min == 0 && tp->tty_termios.c_cc[VTIME] > 0)
            tp->tty_min = 1;
    }

    if (!(tp->tty_termios.c_iflag & IXON)) {
        tp->tty_inhibited = RUNNING;
        tp->tty_events = 1;
    }

    if (tp->tty_termios.c_ospeed == B0) sigchar(tp, SIGHUP, 1);
    (*tp->tty_ioctl)(tp, 0);
}

void sigchar(tty_t *tp, int sig, int mayflush) {
    int status;

    if (tp->tty_pgrp != 0) {
        if (OK != (status = sys_kill(tp->tty_pgrp, sig))) {
            panic("Error; call to sys_kill failed: %d", status);
        }
    }

    if (mayflush && !(tp->tty_termios.c_lflag & NOFLSH)) {
        tp->tty_incount = tp->tty_eotct = 0;
        tp->tty_intail = tp->tty_inhead;
        (*tp->tty_ocancel)(tp, 0);
        tp->tty_inhibited = RUNNING;
        tp->tty_events = 1;
    }
}

static void tty_icancel(tty_t *tp) {
    tp->tty_incount = tp->tty_eotct = 0;
    tp->tty_intail = tp->tty_inhead;
    (*tp->tty_icancel)(tp, 0);
}

static int tty_devnop(tty_t *UNUSED(tp), int UNUSED(try)) {
    return 0;
}

static void tty_init() {
    tty_t *tp;
    int s;

    system_hz = sys_hz();
    memset(tty_table, '\0', sizeof(tty_table));

    for (tp = FIRST_TTY, s = 0; tp < END_TTY; tp++, s++) {
        tp->tty_index = s;
        init_timer(&tp->tty_tmr);

        tp->tty_intail = tp->tty_inhead = tp->tty_inbuf;
        tp->tty_min = 1;
        tp->tty_incaller = tp->tty_outcaller = tp->tty_iocaller = NONE;
        tp->tty_termios = termios_defaults;
        tp->tty_icancel = tp->tty_ocancel = tp->tty_ioctl = tp->tty_close = tp->tty_open = tty_devnop;
        if (tp < tty_addr(NR_CONS)) {
            scr_init(tp);

            kb_init(tp);
            tp->tty_minor = CONS_MINOR + s;
        } else {
            rs_init(tp);
            tp->tty_minor = RS232_MINOR + s - NR_CONS;
        }
    }
}

static void tty_timed_out(int arg) {
    tty_t *tty_ptr = &tty_table[arg];
    tty_ptr->tty_min = 0;
    tty_ptr->tty_events = 1;
}

static void settimer(tty_t *tty_ptr, int enable) {
    clock_t ticks;

    if (enable) {
        ticks = tty_ptr->tty_termios.c_cc[VTIME] * (system_hz / 10);
        set_timer(&tty_ptr->tty_tmr, ticks, tty_timed_out, tty_ptr->tty_index);
    } else {
        cancel_timer(&tty_ptr->tty_tmr);
    }
}
```