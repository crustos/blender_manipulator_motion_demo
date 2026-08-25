/* drive_node.c - the base controller, as firmware.
 *
 * A robotsim robot's base, driven by real C instead of by Python calling
 * drive.drive(). The board reads an encoder, runs a PI position loop and
 * writes a motor duty; robotsim supplies the plant on the other side of those
 * three calls, exactly as a bench would.
 *
 * The seam is hostsim's: sim_encoder_read / sim_motor_write / sim_target are
 * values, not registers, so this proves the control system and says nothing
 * about a PWM driver. That question belongs to armulator.
 *
 * Integer arithmetic throughout. A bare-metal control loop cannot assume an
 * FPU, and integers keep the host build and an ARM build bit-identical.
 *
 *     python3 ../crust/tools/hostsim_build.py boards/drive_node.c
 */

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

#define KP 300L
#define KI 4L
#define DUTY_LIMIT 1000L
#define INTEGRAL_LIMIT 30000L
#define DEADBAND 8L             /* counts; stops hunting about the setpoint */

static long clamp(long v, long limit)
{
    if (v > limit) return limit;
    if (v < -limit) return -limit;
    return v;
}

void kmain(void)
{
    long integral = 0;
    unsigned long f;
    unsigned long period;
    unsigned long next;
    long steps = 0;

    uart_init();
    irq_init();
    timer_start(1000);
    irq_enable();
    uart_puts("[drive] position loop up at 1 kHz\n");

    f = timer_freq();
    period = f / 1000UL;
    next = timer_count() + period;

    /* Bounded so the example terminates; on hardware this never returns. */
    while (steps < 600000) {
        long position;
        long error;
        long duty;

        while (timer_count() < next) {
        }
        next = next + period;

        position = sim_encoder_read();
        error = sim_target() - position;

        if (error > -DEADBAND && error < DEADBAND) {
            /* Inside the deadband, stop winding the integrator up. A loop
             * that keeps integrating a rounding error will eventually push
             * the output off the setpoint it already reached. */
            error = 0;
        } else {
            integral = clamp(integral + error, INTEGRAL_LIMIT);
        }

        duty = clamp((KP * error + KI * integral) / 1000L, DUTY_LIMIT);
        sim_motor_write(duty);

        if ((steps % 2000) == 0) {
            uart_puts("[drive] pos=");
            uart_putdec(position);
            uart_puts(" err=");
            uart_putdec(error);
            uart_puts(" duty=");
            uart_putdec(duty);
            uart_puts("\n");
        }
        steps = steps + 1;
    }
    uart_puts("[drive] done\n");
}
