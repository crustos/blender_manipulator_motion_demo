/* pid_node.cpp - the same loop, in the C++ subset.
 *
 * Lowered to plain C by crust's tools/cpprust.py before hostsim compiles it,
 * so a class method becomes Pid_step(Pid *this, ...) and there is no C++ left
 * by the time anything is built. The front end accepts a subset and *refuses*
 * what it cannot lower rather than guessing -- so if this stops compiling, that
 * is a real answer about the source rather than a gap in the toolchain.
 *
 * Kept deliberately close to drive_node.c: the point is that the class-based
 * version controls the same plant the same way, not that it is better.
 */

extern "C" {
void uart_init(void);
void uart_puts(char *s);
void uart_putdec(long v);
void irq_init(void);
void irq_enable(void);
void timer_start(int hz);
unsigned long timer_count(void);
unsigned long timer_freq(void);

long sim_encoder_read(void);
void sim_motor_write(long duty);
long sim_target(void);
}

class Pid {
public:
    long kp;
    long ki;
    long integral;
    long limit;
    long windup;

    long clamp(long v, long bound) {
        if (v > bound) return bound;
        if (v < -bound) return -bound;
        return v;
    }

    long step(long error) {
        integral = clamp(integral + error, windup);
        return clamp((kp * error + ki * integral) / 1000, limit);
    }
};

extern "C" void kmain(void)
{
    Pid pid;
    unsigned long f;
    unsigned long period;
    unsigned long next;
    long steps = 0;

    pid.kp = 300;
    pid.ki = 4;
    pid.integral = 0;
    pid.limit = 1000;
    pid.windup = 30000;

    uart_init();
    irq_init();
    timer_start(1000);
    irq_enable();
    uart_puts((char *)"[pid] c++ loop up at 1 kHz\n");

    f = timer_freq();
    period = f / 1000;
    next = timer_count() + period;

    while (steps < 600000) {
        while (timer_count() < next) {
        }
        next = next + period;

        long position = sim_encoder_read();
        long error = sim_target() - position;
        long duty = pid.step(error);
        sim_motor_write(duty);

        if ((steps % 2000) == 0) {
            /* Log the duty already computed. Calling step() again here to get
             * a value to print would advance the integrator a second time --
             * a logging line quietly changing the control output. */
            uart_puts((char *)"[pid] pos=");
            uart_putdec(position);
            uart_puts((char *)" duty=");
            uart_putdec(duty);
            uart_puts((char *)"\n");
        }
        steps = steps + 1;
    }
    uart_puts((char *)"[pid] done\n");
}
