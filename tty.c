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
		if (tp->tty_events) {
			handle_events(tp);
		}
	}
}

static void handle_clock_notification(message *tty_mess)
{
    expire_timers(tty_mess->m_notify.timestamp);
}

static void handle_hardware_notification(message *tty_mess)
{
#if NR_RS_LINES > 0
    if (tty_mess->m_notify.interrupts & rs_irq_set)
        rs_interrupt(tty_mess);
#endif
    expire_timers(tty_mess->m_notify.timestamp);
}

static void handle_kernel_notification(message *tty_mess, int ipc_status)
{
    if (!is_ipc_notify(ipc_status))
        return;
        
    switch (_ENDPOINT_P(tty_mess->m_source)) {
    case CLOCK:
        handle_clock_notification(tty_mess);
        break;
    case HARDWARE:
        handle_hardware_notification(tty_mess);
        break;
    default:
        break;
    }
}

static int process_tty_control_messages(message *tty_mess)
{
    if (tty_mess->m_type == TTY_FKEY_CONTROL) {
        do_fkey_ctl(tty_mess);
        return 1;
    }
    return 0;
}

static int process_tty_input_messages(message *tty_mess)
{
    if (tty_mess->m_type == TTY_INPUT_UP || tty_mess->m_type == TTY_INPUT_EVENT) {
        do_input(tty_mess);
        return 1;
    }
    return 0;
}

static int process_non_cdev_request(message *tty_mess, int ipc_status)
{
    if (!IS_CDEV_RQ(tty_mess->m_type)) {
        chardriver_process(&tty_tab, tty_mess, ipc_status);
        return 1;
    }
    return 0;
}

static int process_video_request(message *tty_mess, int ipc_status, int line)
{
    if (line == VIDEO_MINOR) {
        do_video(tty_mess, ipc_status);
        return 1;
    }
    return 0;
}

static int handle_special_message(message *tty_mess, int ipc_status, int line)
{
    if (process_tty_control_messages(tty_mess)) {
        return 1;
    }
    
    if (process_tty_input_messages(tty_mess)) {
        return 1;
    }
    
    if (process_non_cdev_request(tty_mess, ipc_status)) {
        return 1;
    }
    
    if (process_video_request(tty_mess, ipc_status, line)) {
        return 1;
    }
    
    return 0;
}

int main(void)
{
	sef_local_startup();
	run_tty_event_loop();
	return 0;
}

static void run_tty_event_loop(void)
{
	message tty_mess;
	int ipc_status;

	while (TRUE) {
		process_tty_events();
		receive_and_handle_message(&tty_mess, &ipc_status);
	}
}

static void receive_and_handle_message(message *tty_mess, int *ipc_status)
{
	int r = driver_receive(ANY, tty_mess, ipc_status);
	if (r != 0)
		panic("driver_receive failed with: %d", r);

	if (is_ipc_notify(*ipc_status)) {
		handle_kernel_notification(tty_mess, *ipc_status);
		return;
	}

	process_character_driver_message(tty_mess, *ipc_status);
}

static void process_character_driver_message(message *tty_mess, int ipc_status)
{
	int line;
	
	if (OK != chardriver_get_minor(tty_mess, &line))
		return;
		
	if (handle_special_message(tty_mess, ipc_status, line))
		return;

	chardriver_process(&tty_tab, tty_mess, ipc_status);
}

static void set_color(tty_t *tp, int color)
{
	const char ESC_CHAR = '\033';
	const size_t BUFFER_SIZE = 8;
	char buf[BUFFER_SIZE];

	buf[0] = ESC_CHAR;
	snprintf(&buf[1], BUFFER_SIZE - 1, "[1;%dm", color);
	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) buf, BUFFER_SIZE,
		CDEV_NONBLOCK, 0);
}

static void reset_color(tty_t *tp)
{
	char buf[8];

#define SGR_COLOR_RESET	39
#define ESCAPE_CHAR '\033'
	
	buf[0] = ESCAPE_CHAR;
	snprintf(&buf[1], sizeof(buf) - 1, "[0;%dm", SGR_COLOR_RESET);
	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) buf, sizeof(buf),
		CDEV_NONBLOCK, 0);
}

tty_t *line2tty(devminor_t line)
{
	tty_t* tp;

	line = normalize_line(line);

	tp = get_tty_by_line(line);

	if (tp != NULL && !tty_active(tp))
		tp = NULL;

	return tp;
}

static devminor_t normalize_line(devminor_t line)
{
	if (line == CONS_MINOR || line == LOG_MINOR)
		return consoleline;
	return line;
}

static tty_t* get_tty_by_line(devminor_t line)
{
	if (line == VIDEO_MINOR)
		return NULL;
	
	if (is_console_line(line))
		return tty_addr(line - CONS_MINOR);
	
	if (is_rs232_line(line))
		return tty_addr(line - RS232_MINOR + NR_CONS);
	
	return NULL;
}

static int is_console_line(devminor_t line)
{
	return (line - CONS_MINOR) < NR_CONS;
}

static int is_rs232_line(devminor_t line)
{
	return (line - RS232_MINOR) < NR_RS_LINES;
}

static void sef_local_startup(void)
{
	sef_setcb_init_fresh(sef_cb_init_fresh);
	sef_setcb_init_restart(SEF_CB_INIT_RESTART_STATEFUL);
	sef_setcb_signal_handler(sef_cb_signal_handler);
	sef_startup();
}

static int get_kernel_machine(void)
{
	int r = sys_getmachine(&machine);
	if (r != OK) {
		panic("Couldn't obtain kernel environment: %d", r);
	}
	return r;
}

static void load_console_param(void)
{
	char val[CONS_ARG];
	if (env_get_param("console", val, sizeof(val)) == OK) {
		set_console_line(val);
	}
}

static void load_kernel_color_param(void)
{
	char val[CONS_ARG];
	if (env_get_param("kernelclr", val, sizeof(val)) == OK) {
		set_kernel_color(val);
	}
}

static void initialize_subsystems(void)
{
	tty_init();
	kb_init_once();
	sys_diagctl_register();
}

static int sef_cb_init_fresh(int UNUSED(type), sef_init_info_t *UNUSED(info))
{
	get_kernel_machine();
	load_console_param();
	load_kernel_color_param();
	initialize_subsystems();
	
	return OK;
}

static int min_value(int a, int b)
{
    return a < b ? a : b;
}

static int check_console_match(const char *term, const char *pattern, size_t pattern_size)
{
    return !strncmp(term, pattern, min_value(CONS_ARG - 1, pattern_size - 1));
}

static int check_numbered_device(const char *term, const char *prefix, int index)
{
    char device[6];
    strlcpy(device, prefix, sizeof(device));
    device[4] += index;
    return check_console_match(term, device, sizeof(device));
}

static void check_tty_consoles(const char *term)
{
    int i;
    for (i = 1; i < NR_CONS; i++) {
        if (check_numbered_device(term, "ttyc0", i)) {
            consoleline = CONS_MINOR + i;
            return;
        }
    }
}

static void check_serial_consoles(const char *term)
{
    int i;
    assert(NR_RS_LINES <= 9);
    for (i = 0; i < NR_RS_LINES; i++) {
        if (check_numbered_device(term, "tty00", i)) {
            consoleline = RS232_MINOR + i;
            return;
        }
    }
}

static void set_console_line(char term[CONS_ARG])
{
    if (check_console_match(term, "console", 8)) {
        consoleline = CONS_MINOR + 0;
        return;
    }
    
    check_tty_consoles(term);
    check_serial_consoles(term);
}

static void set_kernel_color(char color[CONS_ARG])
{
	#define SGR_COLOR_START	30
	#define SGR_COLOR_END	37
	#define MIN_COLOR_OFFSET 0
	#define MAX_COLOR_OFFSET 7

	int def_color = atoi(color);
	
	if (def_color >= MIN_COLOR_OFFSET && def_color <= MAX_COLOR_OFFSET) {
		kernel_msg_color = def_color + SGR_COLOR_START;
	}
}

static void copy_kernel_messages(char *kernel_buf_copy, struct kmessages *kmess_ptr, 
                                 int prev_next, int next, int bytes)
{
	int copy = MIN(_KMESS_BUF_SIZE - prev_next, bytes);
	memcpy(kernel_buf_copy, &kmess_ptr->km_buf[prev_next], copy);

	if (copy < bytes) {
		memcpy(&kernel_buf_copy[copy], &kmess_ptr->km_buf[0], bytes - copy);
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

	if (kernel_msg_color != 0)
		set_color(tp, kernel_msg_color);
		
	do_write(tp->tty_minor, 0, KERNEL, (cp_grant_id_t) kernel_buf_copy, 
	         bytes, CDEV_NONBLOCK, 0);
	         
	if (kernel_msg_color != 0)
		reset_color(tp);
		
	if (restore) {
		*tp = rtp;
	}

	prev_next = next;
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
	}
}

static ssize_t do_read(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp;
	int r;

	if ((tp = line2tty(minor)) == NULL)
		return ENXIO;

	if (tp->tty_incaller != NONE || tp->tty_inleft > 0)
		return EIO;
	if (size <= 0)
		return EINVAL;

	setup_read_request(tp, endpt, id, grant, size);
	handle_vtime_settings(tp);
	perform_read_transfer(tp);
	
	if (tp->tty_inleft == 0)
		return EDONTREPLY;

	if (flags & CDEV_NONBLOCK)
		return handle_nonblocking_read(tp);

	if (tp->tty_select_ops)
		select_retry(tp);

	return EDONTREPLY;
}

static void setup_read_request(tty_t *tp, endpoint_t endpt, cdev_id_t id,
	cp_grant_id_t grant, size_t size)
{
	tp->tty_incaller = endpt;
	tp->tty_inid = id;
	tp->tty_ingrant = grant;
	assert(tp->tty_incum == 0);
	tp->tty_inleft = size;
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

static void perform_read_transfer(tty_t *tp)
{
	in_transfer(tp);
	handle_events(tp);
}

static ssize_t handle_nonblocking_read(tty_t *tp)
{
	int r;
	
	tty_icancel(tp);
	r = tp->tty_incum > 0 ? tp->tty_incum : EAGAIN;
	tp->tty_inleft = tp->tty_incum = 0;
	tp->tty_incaller = NONE;
	return r;
}

static ssize_t validate_write_params(tty_t *tp, size_t size)
{
	if (tp == NULL)
		return ENXIO;
	if (tp->tty_outcaller != NONE || tp->tty_outleft > 0)
		return EIO;
	if (size <= 0)
		return EINVAL;
	return 0;
}

static void setup_write_operation(tty_t *tp, endpoint_t endpt, cdev_id_t id,
	cp_grant_id_t grant, size_t size)
{
	tp->tty_outcaller = endpt;
	tp->tty_outid = id;
	tp->tty_outgrant = grant;
	assert(tp->tty_outcum == 0);
	tp->tty_outleft = size;
}

static ssize_t handle_nonblocking_write(tty_t *tp)
{
	ssize_t r = tp->tty_outcum > 0 ? tp->tty_outcum : EAGAIN;
	tp->tty_outleft = 0;
	tp->tty_outcum = 0;
	tp->tty_outcaller = NONE;
	return r;
}

static ssize_t do_write(devminor_t minor, u64_t UNUSED(position),
	endpoint_t endpt, cp_grant_id_t grant, size_t size, int flags,
	cdev_id_t id)
{
	tty_t *tp = line2tty(minor);
	ssize_t r = validate_write_params(tp, size);
	
	if (r != 0)
		return r;

	setup_write_operation(tp, endpt, id, grant, size);
	handle_events(tp);
	
	if (tp->tty_outleft == 0)
		return EDONTREPLY;

	if (flags & CDEV_NONBLOCK)
		return handle_nonblocking_write(tp);

	if (tp->tty_select_ops)
		select_retry(tp);

	return EDONTREPLY;
}

static int handle_termios_get(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	const size_t TERMIOS_SIZE = sizeof(struct termios);
	const size_t OFFSET = 0;
	
	return sys_safecopyto(endpt, grant, OFFSET, (vir_bytes) &tp->tty_termios,
		TERMIOS_SIZE);
}

static int handle_termios_set(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp,
                              unsigned long request)
{
	int r;
	
	if (request == TIOCSETAF) 
		tty_icancel(tp);
		
	r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &tp->tty_termios,
		sizeof(struct termios));
	if (r != OK) 
		return r;
		
	setattr(tp);
	return OK;
}

static int handle_flush(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
	int flush_flags;
	int r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &flush_flags, sizeof(flush_flags));
	if (r != OK) 
		return r;
		
	if (flush_flags & FREAD) 
		tty_icancel(tp);
	if (flush_flags & FWRITE) 
		(*tp->tty_ocancel)(tp, 0);
		
	return OK;
}

static int get_window_size(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
    return sys_safecopyto(endpt, grant, 0, (vir_bytes) &tp->tty_winsize,
        sizeof(struct winsize));
}

static int set_window_size(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp)
{
    int r = sys_safecopyfrom(endpt, grant, 0, (vir_bytes) &tp->tty_winsize,
        sizeof(struct winsize));
    if (r == OK)
        sigchar(tp, SIGWINCH, 0);
    return r;
}

static int handle_winsize(endpoint_t endpt, cp_grant_id_t grant, tty_t *tp,
                         unsigned long request)
{
    if (request == TIOCGWINSZ) {
        return get_window_size(endpt, grant, tp);
    }
    return set_window_size(endpt, grant, tp);
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
		
	ticks = calculate_bell_duration(&bell);
		
	beep_x(bell.kb_pitch, ticks);
	return OK;
}

static clock_t calculate_bell_duration(const kio_bell_t *bell)
{
	#define MICROSECONDS_PER_SECOND 1000000
	
	clock_t ticks;
	
	ticks = bell->kb_duration.tv_usec * system_hz / MICROSECONDS_PER_SECOND;
	ticks += bell->kb_duration.tv_sec * system_hz;
	
	if (!ticks)
		ticks++;
		
	return ticks;
}

static int handle_output_wait(tty_t *tp, int flags, endpoint_t endpt, 
    cdev_id_t id, unsigned long request, cp_grant_id_t grant)
{
    if (tp->tty_outleft > 0) {
        if (flags & CDEV_NONBLOCK)
            return EAGAIN;
        tp->tty_iocaller = endpt;
        tp->tty_ioid = id;
        tp->tty_ioreq = request;
        tp->tty_iogrant = grant;
        return EDONTREPLY;
    }
    return OK;
}

static int copy_int_to_user(endpoint_t endpt, cp_grant_id_t grant, int value)
{
    return sys_safecopyto(endpt, grant, 0, (vir_bytes) &value, sizeof(value));
}

static int handle_flow_control(tty_t *tp, int inhibit)
{
    tp->tty_inhibited = inhibit;
    tp->tty_events = 1;
    return OK;
}

static int handle_break_signal(tty_t *tp, int on)
{
    if (on && tp->tty_break_on != NULL) {
        (*tp->tty_break_on)(tp, 0);
    } else if (!on && tp->tty_break_off != NULL) {
        (*tp->tty_break_off)(tp, 0);
    }
    return OK;
}

static int handle_console_operation(tty_t *tp, endpoint_t endpt, 
    cp_grant_id_t grant, unsigned long request)
{
    if (!isconsole(tp))
        return OK;
    
    if (request == KIOCSMAP)
        return kbd_loadmap(endpt, grant);
    if (request == TIOCSFON)
        return con_loadfont(endpt, grant);
    
    return OK;
}

#define TTYDISC 0
#define TTY_IN_BYTES 1024

static int do_ioctl(devminor_t minor, unsigned long request, endpoint_t endpt,
    cp_grant_id_t grant, int flags, endpoint_t user_endpt, cdev_id_t id)
{
    tty_t *tp;
    int r;

    if ((tp = line2tty(minor)) == NULL)
        return ENXIO;

    switch (request) {
    case TIOCGETA:
        return handle_termios_get(endpt, grant, tp);

    case TIOCSETAW:
    case TIOCSETAF:
        r = handle_output_wait(tp, flags, endpt, id, request, grant);
        if (r != OK)
            return r;
        return handle_termios_set(endpt, grant, tp, request);

    case TIOCDRAIN:
        return handle_output_wait(tp, flags, endpt, id, request, grant);

    case TIOCSETA:
        return handle_termios_set(endpt, grant, tp, request);

    case TIOCFLUSH:
        return handle_flush(endpt, grant, tp);
        
    case TIOCSTART:
        return handle_flow_control(tp, 0);
        
    case TIOCSTOP:
        return handle_flow_control(tp, 1);
        
    case TIOCSBRK:
        return handle_break_signal(tp, 1);
        
    case TIOCCBRK:
        return handle_break_signal(tp, 0);

    case TIOCGWINSZ:
    case TIOCSWINSZ:
        return handle_winsize(endpt, grant, tp, request);
        
    case KIOCBELL:
        return handle_bell(endpt, grant, tp);
        
    case TIOCGETD:
        return copy_int_to_user(endpt, grant, TTYDISC);
        
    case TIOCSETD:
        printf("TTY: TIOCSETD: can't set any other line discipline.\n");
        return ENOTTY;
        
    case TIOCGLINED:
        return sys_safecopyto(endpt, grant, 0, (vir_bytes) lined, sizeof(lined));
        
    case TIOCGQSIZE:
        return copy_int_to_user(endpt, grant, TTY_IN_BYTES);
        
    case KIOCSMAP:
    case TIOCSFON:
        return handle_console_operation(tp, endpt, grant, request);

    case TIOCSCTTY:
        tp->tty_pgrp = user_endpt;
        return OK;
        
    case TIOCGPGRP:
    case TIOCSPGRP:
    default:
        return ENOTTY;
    }
}

static int is_log_console_read_access(devminor_t minor, int access, tty_t *tp)
{
	return (minor == LOG_MINOR && isconsole(tp) && (access & CDEV_R_BIT));
}

static int handle_regular_tty_open(tty_t *tp, int access, endpoint_t user_endpt)
{
	int r = OK;
	
	if (!(access & CDEV_NOCTTY)) {
		tp->tty_pgrp = user_endpt;
		r = CDEV_CTTY;
	}
	
	tp->tty_openct++;
	if (tp->tty_openct == 1) {
		(*tp->tty_open)(tp, 0);
	}
	
	return r;
}

static int do_open(devminor_t minor, int access, endpoint_t user_endpt)
{
	tty_t *tp;

	if ((tp = line2tty(minor)) == NULL)
		return ENXIO;

	if (is_log_console_read_access(minor, access, tp))
		return EACCES;

	if (minor != LOG_MINOR || !isconsole(tp))
		return handle_regular_tty_open(tp, access, user_endpt);

	return OK;
}

static int is_closable_device(devminor_t minor, tty_t *tp)
{
	return (minor != LOG_MINOR || !isconsole(tp));
}

static void reset_tty_settings(tty_t *tp)
{
	tp->tty_pgrp = 0;
	tp->tty_termios = termios_defaults;
	tp->tty_winsize = winsize_defaults;
}

static void cancel_tty_operations(tty_t *tp)
{
	tty_icancel(tp);
	(*tp->tty_ocancel)(tp, 0);
	(*tp->tty_close)(tp, 0);
}

static void close_tty_device(tty_t *tp)
{
	reset_tty_settings(tp);
	cancel_tty_operations(tp);
	setattr(tp);
}

static int do_close(devminor_t minor)
{
	tty_t *tp;

	if ((tp = line2tty(minor)) == NULL)
		return ENXIO;

	if (is_closable_device(minor, tp) && --tp->tty_openct == 0) {
		close_tty_device(tp);
	}

	return OK;
}

static int cancel_input_operation(tty_t *tp)
{
    tty_icancel(tp);
    int result = tp->tty_incum > 0 ? tp->tty_incum : EAGAIN;
    tp->tty_inleft = 0;
    tp->tty_incum = 0;
    tp->tty_incaller = NONE;
    return result;
}

static int cancel_output_operation(tty_t *tp)
{
    int result = tp->tty_outcum > 0 ? tp->tty_outcum : EAGAIN;
    tp->tty_outleft = 0;
    tp->tty_outcum = 0;
    tp->tty_outcaller = NONE;
    return result;
}

static int cancel_io_request(tty_t *tp)
{
    tp->tty_ioreq = 0;
    tp->tty_iocaller = NONE;
    return EINTR;
}

static int is_input_match(tty_t *tp, endpoint_t endpt, cdev_id_t id)
{
    return tp->tty_inleft != 0 && endpt == tp->tty_incaller && id == tp->tty_inid;
}

static int is_output_match(tty_t *tp, endpoint_t endpt, cdev_id_t id)
{
    return tp->tty_outleft != 0 && endpt == tp->tty_outcaller && id == tp->tty_outid;
}

static int is_io_request_match(tty_t *tp, endpoint_t endpt, cdev_id_t id)
{
    return tp->tty_ioreq != 0 && endpt == tp->tty_iocaller && id == tp->tty_ioid;
}

static int do_cancel(devminor_t minor, endpoint_t endpt, cdev_id_t id)
{
    tty_t *tp;
    int r;

    if ((tp = line2tty(minor)) == NULL)
        return ENXIO;

    r = EDONTREPLY;
    
    if (is_input_match(tp, endpt, id)) {
        r = cancel_input_operation(tp);
    } else if (is_output_match(tp, endpt, id)) {
        r = cancel_output_operation(tp);
    } else if (is_io_request_match(tp, endpt, id)) {
        r = cancel_io_request(tp);
    }
    
    if (r != EDONTREPLY)
        tp->tty_events = 1;
        
    return r;
}

int select_try(struct tty *tp, int ops)
{
	int ready_ops = 0;

	if (tp->tty_termios.c_ospeed == B0) {
		return ops;
	}

	if (ops & CDEV_OP_RD) {
		ready_ops |= check_read_ready(tp);
	}

	if (ops & CDEV_OP_WR) {
		ready_ops |= check_write_ready(tp);
	}
	
	return ready_ops;
}

static int check_read_ready(struct tty *tp)
{
	if (tp->tty_inleft > 0) {
		return CDEV_OP_RD;
	}
	
	if (tp->tty_incount > 0 && is_read_allowed(tp)) {
		return CDEV_OP_RD;
	}
	
	return 0;
}

static int check_write_ready(struct tty *tp)
{
	if (tp->tty_outleft > 0) {
		return CDEV_OP_WR;
	}
	
	if ((*tp->tty_devwrite)(tp, 1)) {
		return CDEV_OP_WR;
	}
	
	return 0;
}

static int is_read_allowed(struct tty *tp)
{
	return !(tp->tty_termios.c_lflag & ICANON) || tp->tty_eotct > 0;
}

int select_retry(struct tty *tp)
{
	int ops;

	if (!tp->tty_select_ops) {
		return OK;
	}

	ops = select_try(tp, tp->tty_select_ops);
	if (!ops) {
		return OK;
	}

	chardriver_reply_select(tp->tty_select_proc, tp->tty_select_minor, ops);
	tp->tty_select_ops &= ~ops;
	
	return OK;
}

static int validate_select_operation(tty_t *tp, devminor_t minor)
{
	if (tp->tty_select_ops != 0 && tp->tty_select_minor != minor) {
		printf("TTY: select on one object with two minors (%d, %d)\n",
			tp->tty_select_minor, minor);
		return EBADF;
	}
	return 0;
}

static void register_select_watch(tty_t *tp, unsigned int ops, endpoint_t endpt, devminor_t minor)
{
	tp->tty_select_ops |= ops;
	tp->tty_select_proc = endpt;
	tp->tty_select_minor = minor;
}

static int do_select(devminor_t minor, unsigned int ops, endpoint_t endpt)
{
	#define SELECT_OPS_MASK (CDEV_OP_RD | CDEV_OP_WR | CDEV_OP_ERR)
	
	tty_t *tp;
	int ready_ops, watch;
	int validation_result;

	if ((tp = line2tty(minor)) == NULL)
		return ENXIO;

	watch = (ops & CDEV_NOTIFY);
	ops &= SELECT_OPS_MASK;

	ready_ops = select_try(tp, ops);
	ops &= ~ready_ops;
	
	if (!ops || !watch)
		return ready_ops;

	validation_result = validate_select_operation(tp, minor);
	if (validation_result != 0)
		return validation_result;

	register_select_watch(tp, ops, endpt, minor);

	return ready_ops;
}

void handle_events(tty_t *tp)
{
    process_device_events(tp);
    in_transfer(tp);
    check_and_handle_input_ready(tp);
}

static void process_device_events(tty_t *tp)
{
    do {
        tp->tty_events = 0;
        (*tp->tty_devread)(tp, 0);
        (*tp->tty_devwrite)(tp, 0);
        if (tp->tty_ioreq != 0) 
            dev_ioctl(tp);
    } while (tp->tty_events);
}

static void check_and_handle_input_ready(tty_t *tp)
{
    if (tp->tty_incum >= tp->tty_min && tp->tty_inleft > 0) {
        tty_reply(tp->tty_inrepcode, tp->tty_incaller, tp->tty_inproc, tp->tty_incum);
        tp->tty_inleft = 0;
    }
}