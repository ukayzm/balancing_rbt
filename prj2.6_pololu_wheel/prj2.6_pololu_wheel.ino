#include "board.h"
#include "wheel.h"

extern void setup_IR(void);
extern uint32_t recv_IR(void);

Wheel wheel_left(&count_m0, &total_count_m0, M0_PWM_PIN, M0_CTRL0_PIN, M0_CTRL1_PIN, INITIAL_PWM_M0, MIN_PWM_M0, MIN_SPEED_M0);
Wheel wheel_right(&count_m1, &total_count_m1, M1_PWM_PIN, M1_CTRL0_PIN, M1_CTRL1_PIN, INITIAL_PWM_M1, MIN_PWM_M1, MIN_SPEED_M1);

#define LOOP_MS     10
unsigned long loop_timer;

void do_test1(Wheel *w0, Wheel *w1)
{
	int16_t pwm;
	double speed;
	int i;

	/* forward */
	for (pwm = -260; pwm <= 260; pwm += 10) {
		Serial.print("pwm ");	Serial.print(pwm);	Serial.print(": ");
		w0->SetPwm(pwm);
		if (w1) w1->SetPwm(pwm);
		for (i = 0; i < 100; i++) {
			delay(10);
			w0->Update();
			if (w1) w1->Update();
		}
		speed = w0->GetCurSpeed();
		Serial.print(speed, 2); Serial.print(" mm/s ");
		if (w1) speed = w1->GetCurSpeed();
		Serial.print(speed, 2); Serial.println(" mm/s");
	}

	/* backward */
	for (pwm = 260; pwm >= -260; pwm -= 10) {
		Serial.print("pwm ");	Serial.print(pwm);	Serial.print(": ");
		w0->SetPwm(pwm);
		if (w1) w1->SetPwm(pwm);
		for (i = 0; i < 100; i++) {
			delay(10);
			w0->Update();
			if (w1) w1->Update();
		}
		speed = w0->GetCurSpeed();
		Serial.print(speed, 2); Serial.print(" mm/s ");
		if (w1) speed = w1->GetCurSpeed();
		Serial.print(speed, 2); Serial.println(" mm/s");
	}

	w0->SetPwm(0);
	if (w1) w1->SetPwm(0);
	w0->Update();
	if (w1) w1->Update();
}

void do_test_wheel(Wheel *w)
{
	int i;

	w->bDiag = 1;

	for (i = 0; i < 100; i++) {
		w->SetPwm(w->GetMinPwm());
		delay(10);
		w->Update();
	}

	for (i = 0; i < 100; i++) {
		w->SetPwm(w->GetInitPwm());
		delay(10);
		w->Update();
#if 1
		if (w->GetCurSpeed() > w->GetMinSpeed()) {
			break;
		}
#endif
	}

	for (i = 0; i < 100; i++) {
		w->SetPwm(w->GetMinPwm());
		delay(10);
		w->Update();
	}

	for (i = 0; i < 100; i++) {
		w->SetPwm(0);
		delay(10);
		w->Update();
	}
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  setup_IR();

  setup_board();

  //do_test1(&wheel_left, &wheel_right);
  Serial.println("start...");
}

uint32_t last_ir_code;
uint32_t last_ir_ms;

void loop()
{
  uint32_t ir_code;

  ir_code = recv_IR();

  if (ir_code) {
    if ((last_ir_ms + 250 < millis()) || (ir_code != last_ir_code)) {
      if (ir_code == 0x4de93dc4) {
        Serial.println("^");
      } else if (ir_code == 0x26e6c1ca) {
        Serial.println("v");
      } else if (ir_code == 0xdad4e90b) {
        Serial.println("<");
  		do_test_wheel(&wheel_left);
      } else if (ir_code == 0x6d89e538) {
        Serial.println(">");
  		do_test_wheel(&wheel_right);
      } else if (ir_code == 0x7d399127) {
        Serial.println("OK");
      } else if (ir_code == 0x68a199f0) {
        Serial.println("REC");
      } else if (ir_code == 0xf169e8b2) {
        Serial.println("REPLAY");
      } else if (ir_code == 0xcf98a7b6) {
        Serial.println("CH+");
      } else if (ir_code == 0x107f5e27) {
        Serial.println("CH-");
      } else if (ir_code == 0xd7d018ec) {
        Serial.println("VOL+");
      } else if (ir_code == 0xf49b208a) {
        Serial.println("VOL-");
      } else if (ir_code == 0x32939470) {
        Serial.println("PLAY/PAUSE");
      } else if (ir_code == 0x16d5cb04) {
        Serial.println("FF");
      } else if (ir_code == 0x19fd189b) {
        Serial.println("REW");
      } else if (ir_code == 0x407e2e01) {
        Serial.println("STOP");
      } else if (ir_code == 0x7547960e) {
        Serial.println("NEXT");
      } else if (ir_code == 0xd1921028) {
        Serial.println("PREV");
      } else if (ir_code == 0x26ecbcf3) {
        Serial.println("(0)");
      } else if (ir_code == 0x9004b206) {
        Serial.println("(1)");
  		do_test1(&wheel_left, NULL);
      } else if (ir_code == 0xc35f14b9) {
        Serial.println("(2)");
  		do_test1(&wheel_left, &wheel_right);
      } else if (ir_code == 0xa6034632) {
        Serial.println("(3)");
  		do_test1(&wheel_right, NULL);
      } else if (ir_code == 0x45897fb8) {
        Serial.println("(4)");
      } else if (ir_code == 0x6a8bf890) {
        Serial.println("(5)");
      } else if (ir_code == 0x08a2cf97) {
        Serial.println("(6)");
      } else if (ir_code == 0x462c837e) {
        Serial.println("(7)");
      } else if (ir_code == 0x42c5c050) {
        Serial.println("(8)");
      } else if (ir_code == 0x67c83928) {
        Serial.println("(9)");
      } else if (ir_code == 0x6bef8366) {
        Serial.println("PWR");
      } else if (ir_code == 0xbe663d0a) {
        Serial.println("MUTE");
      } else if (ir_code == 0xae7fbf1d) {
        Serial.println("TEXT");
      } else {
        Serial.println(ir_code, HEX);
      }
	}
    last_ir_ms = millis();
	last_ir_code = ir_code;
  }

  delay(10);
}

