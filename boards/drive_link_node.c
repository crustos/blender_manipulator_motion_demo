/* drive_link_node.c - the base controller, taking orders over the link.
 *
 * Same PI position loop as drive_node.c, but the setpoint arrives as a message
 * from another board instead of from sim_target(). That is the shape a real
 * robot has: a navigation board decides where to go, a motor board makes it
 * happen, and they are separate parts that can disagree.
 *
 * Messages are text, one per link frame:
 *
 *     T<counts>    set the target position
 *     P<counts>    reported position, sent back every report interval
 *
 * link_send returns a status because it can fail. Firmware that ignores it
 * loses messages exactly as it would on a real link, so this counts refusals
 * rather than assuming they cannot happen.
 */

void uart_init(void);
void uart_puts(char *s);
void uart_putdec(long v);
void irq_init(void);
void irq_enable(void);
void timer_start(int hz);
unsigned long timer_count(void);
unsigned long timer_freq(void);

int link_send(const char *data, unsigned long n);
long link_recv(char *out, unsigned long max);

long sim_encoder_read(void);
void sim_motor_write(long duty);

#define KP 300L
#define KI 4L
#define DUTY_LIMIT 1000L
#define INTEGRAL_LIMIT 30000L
#define DEADBAND 8L
#define REPORT_MS 500L

static long clamp(long v, long limit)
{
    if (v > limit) return limit;
    if (v < -limit) return -limit;
    return v;
}

/* Parse a signed decimal after the command letter. No stdlib on a board. */
static long parse_long(const char *s, long n)
{
    long value = 0;
    long i = 1;
    long sign = 1;

    if (i < n && s[i] == '-') {
        sign = -1;
        i = i + 1;
    }
    while (i < n && s[i] >= '0' && s[i] <= '9') {
        value = value * 10 + (long)(s[i] - '0');
        i = i + 1;
    }
    return value * sign;
}

static long format_long(char *out, char tag, long value)
{
    char digits[20];
    long n = 0;
    long i = 0;
    long len = 0;

    out[len] = tag;
    len = len + 1;
    if (value < 0) {
        out[len] = '-';
        len = len + 1;
        value = -value;
    }
    if (value == 0) {
        digits[n] = '0';
        n = 1;
    }
    while (value > 0) {
        digits[n] = (char)('0' + (value % 10));
        value = value / 10;
        n = n + 1;
    }
    for (i = 0; i < n; i = i + 1) {
        out[len] = digits[n - 1 - i];
        len = len + 1;
    }
    return len;
}

void kmain(void)
{
    char inbox[64];
    char outbox[32];
    long integral = 0;
    long target = 0;
    long steps = 0;
    long refused = 0;
    unsigned long f;
    unsigned long period;
    unsigned long next;

    uart_init();
    irq_init();
    timer_start(1000);
    irq_enable();
    uart_puts("[drive] link controller up\n");

    f = timer_freq();
    period = f / 1000UL;
    next = timer_count() + period;

    while (steps < 600000) {
        long position;
        long error;
        long duty;
        long got;

        while (timer_count() < next) {
        }
        next = next + period;

        /* Drain the link: several messages may have arrived in one step. */
        got = link_recv(inbox, sizeof(inbox));
        while (got > 0) {
            if (inbox[0] == 'T') {
                long wanted = parse_long(inbox, got);
                if (wanted != target) {
                    target = wanted;
                    /* A new setpoint invalidates the accumulated history. */
                    integral = 0;
                    uart_puts("[drive] target=");
                    uart_putdec(target);
                    uart_puts("\n");
                }
            }
            got = link_recv(inbox, sizeof(inbox));
        }

        position = sim_encoder_read();
        error = target - position;

        if (error > -DEADBAND && error < DEADBAND) {
            error = 0;
        } else {
            integral = clamp(integral + error, INTEGRAL_LIMIT);
        }
        duty = clamp((KP * error + KI * integral) / 1000L, DUTY_LIMIT);
        sim_motor_write(duty);

        if ((steps % REPORT_MS) == 0) {
            long n = format_long(outbox, 'P', position);
            if (link_send(outbox, (unsigned long)n) != 0) {
                /* The link refused it. Counted rather than retried: a
                 * controller that blocks on telemetry stops controlling. */
                refused = refused + 1;
            }
        }
        steps = steps + 1;
    }

    uart_puts("[drive] refused=");
    uart_putdec(refused);
    uart_puts("\n");
}
