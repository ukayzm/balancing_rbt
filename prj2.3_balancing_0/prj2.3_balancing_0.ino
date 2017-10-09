extern void balancing_setup(void);
extern void balancing_loop(void);
extern void balancing_inc_kp(void);
extern void balancing_inc_ki(void);
extern void balancing_inc_kd(void);
extern void balancing_dec_kp(void);
extern void balancing_dec_ki(void);
extern void balancing_dec_kd(void);
extern void balancing_inc_cal(void);
extern void balancing_dec_cal(void);
extern void balancing_inc_tgt(void);
extern void balancing_dec_tgt(void);
extern void balancing_inc_dir(void);
extern void balancing_dec_dir(void);
extern void balancing_reset_tgtdir(void);
extern void reset_calibration(void);

#define LOOP_MS     10
unsigned long loop_timer;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  setup_IR();

  balancing_setup();

  /*Set the loop_timer variable at the next end loop time */
  loop_timer = micros() + LOOP_MS * 1000;
  while (loop_timer > micros());
  loop_timer += LOOP_MS * 1000;
}

unsigned long prev_total_count_m3, prev_total_count_m4;

void loop()
{
  check_ir();
  balancing_loop();

  /*
   * The angle calculations are tuned for a loop time of LOOP_MS milliseconds.
   * To make sure every loop is exactly LOOP_MS milliseconds a wait loop
   * is created by setting the loop_timer variable to +4000 microseconds
   * every loop.
   */
  while (loop_timer > micros());
  loop_timer += LOOP_MS * 1000;
}

void check_ir()
{
  uint32_t ir_code;
  static uint32_t last_ir_code;
  static uint32_t last_ir_ms;

  ir_code = recv_IR();

  if (ir_code == 0) {
    return;
  }
  if (ir_code != 0x4de93dc4
   && ir_code != 0x26e6c1ca
   && ir_code != 0x6d89e538
   && ir_code != 0xdad4e90b) {
    if ((last_ir_ms + 250 > millis()) && (ir_code == last_ir_code)) {
      return;
    }
  }

  if (ir_code == 0x4de93dc4) {
    Serial.println("^");
	balancing_inc_tgt();
  } else if (ir_code == 0x26e6c1ca) {
    Serial.println("v");
	balancing_dec_tgt();
  } else if (ir_code == 0x6d89e538) {
    Serial.println(">");
	balancing_inc_dir();
  } else if (ir_code == 0xdad4e90b) {
    Serial.println("<");
	balancing_dec_dir();
  } else if (ir_code == 0x7d399127) {
    Serial.println("OK");
	balancing_reset_tgtdir();
  } else if (ir_code == 0x68a199f0) {
    Serial.println("REC");
  } else if (ir_code == 0xf169e8b2) {
    Serial.println("REPLAY");
  } else if (ir_code == 0xcf98a7b6) {
    Serial.println("CH+");
	balancing_inc_kp();
  } else if (ir_code == 0x107f5e27) {
    Serial.println("CH-");
	balancing_dec_kp();
  } else if (ir_code == 0xd7d018ec) {
    Serial.println("VOL+");
	balancing_inc_ki();
  } else if (ir_code == 0xf49b208a) {
    Serial.println("VOL-");
	balancing_dec_ki();
  } else if (ir_code == 0x32939470) {
    Serial.println("PLAY/PAUSE");
  } else if (ir_code == 0x16d5cb04) {
    Serial.println("FF");
	balancing_inc_kd();
  } else if (ir_code == 0x19fd189b) {
    Serial.println("REW");
	balancing_dec_kd();
  } else if (ir_code == 0x407e2e01) {
    Serial.println("STOP");
	reset_calibration();
  } else if (ir_code == 0x7547960e) {
    Serial.println("NEXT");
	balancing_inc_tgt();
  } else if (ir_code == 0xd1921028) {
    Serial.println("PREV");
	balancing_dec_tgt();
  } else if (ir_code == 0x26ecbcf3) {
    Serial.println("(0)");
  } else if (ir_code == 0x9004b206) {
    Serial.println("(1)");
  } else if (ir_code == 0xc35f14b9) {
    Serial.println("(2)");
  } else if (ir_code == 0x6bef8366) {
    Serial.println("PWR");
  } else if (ir_code == 0xbe663d0a) {
    Serial.println("MUTE");
  } else if (ir_code == 0xae7fbf1d) {
    Serial.println("TEXT");
  } else {
    Serial.println(ir_code, HEX);
  }
  last_ir_ms = millis();
  last_ir_code = ir_code;
}
