#include "sys/uart_parser.h"
#include "core/cmd.h"

/* Mailboxes — kept so other modules compile; unused until Part 2 */
atomic_t mailbox_drivetrain = ATOMIC_INIT(127 << 16);
atomic_t mailbox_steering   = ATOMIC_INIT(0);
atomic_t mailbox_blinkers   = ATOMIC_INIT(0);

volatile uint32_t rx_byte_count       = 0;
volatile uint32_t last_cmd_time       = 0;
volatile uint32_t parse_success_count = 0;

int uart_parser_init(void)
{
    /* UART ISR parser removed for Lab 3 Part 1 — will be re-added in Part 2 */
    return 0;
}
