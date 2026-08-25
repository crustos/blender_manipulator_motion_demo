/* nav_node.c - the board that decides where to go.
 *
 * Issues position setpoints to the drive board and listens to what comes back.
 * It never touches a motor: the split between deciding and actuating is the
 * whole reason there are two boards, and it is where a real robot's bugs live
 * -- a nav board that commands faster than the drive board can follow, or one
 * that keeps commanding after telemetry has gone quiet.
 *
 *     T<counts>    sent: go to this position
 *     P<counts>    received: where the drive board thinks it is
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

#define LEG_MS 4000L

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
    long legs[4];
    long leg = 0;
    long steps = 0;
    long heard = 0;
    long silent_ms = 0;
    unsigned long f;
    unsigned long period;
    unsigned long next;

    legs[0] = 2000;
    legs[1] = 4000;
    legs[2] = 1000;
    legs[3] = 3000;

    uart_init();
    irq_init();
    timer_start(1000);
    irq_enable();
    uart_puts("[nav] planner up\n");

    f = timer_freq();
    period = f / 1000UL;
    next = timer_count() + period;

    while (steps < 600000) {
        long got;

        while (timer_count() < next) {
        }
        next = next + period;

        if ((steps % LEG_MS) == 0) {
            long which = (steps / LEG_MS) % 4;
            long n = format_long(outbox, 'T', legs[which]);
            link_send(outbox, (unsigned long)n);
            leg = legs[which];
            uart_puts("[nav] leg -> ");
            uart_putdec(leg);
            uart_puts("\n");
        }

        got = link_recv(inbox, sizeof(inbox));
        if (got > 0) {
            while (got > 0) {
                if (inbox[0] == 'P') {
                    heard = heard + 1;
                }
                got = link_recv(inbox, sizeof(inbox));
            }
            silent_ms = 0;
        } else {
            silent_ms = silent_ms + 1;
            /* Telemetry has stopped. A planner that keeps issuing setpoints
             * into silence is the failure worth catching, so say so. */
            if (silent_ms == 2000) {
                uart_puts("[nav] link silent\n");
            }
        }
        steps = steps + 1;
    }

    uart_puts("[nav] reports=");
    uart_putdec(heard);
    uart_puts("\n");
}
