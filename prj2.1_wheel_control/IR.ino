#include <IRremote.h>

int RECV_PIN = A3;

IRrecv irrecv(RECV_PIN);

void setup_IR()
{
  irrecv.enableIRIn(); // Start the receiver
}

uint32_t recv_IR()
{
  decode_results results;
  results.value = 0;
  if (irrecv.decode(&results)) {
    irrecv.resume(); // Receive the next value
  }
  return results.value;
}
