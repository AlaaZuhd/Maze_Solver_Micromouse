int	speed		= 0,		/* Y-Axis read of the Joystick */
	speedA		= 0,		/* Speed to be written to the right motor */
	speedB		= 0,		/* Speed to be written to the left motor */
	direction	= 0,		/* X-Axis read of the Joystick */
	max_speed	= 0;		/* The speed of the faster motor */

void setup () {
	pinMode(A0,  INPUT);	/* Joystick X-Axis input */
	pinMode(A1,  INPUT);	/* Joystick Y-Axis input */
	pinMode(11, OUTPUT);	/* Left Motor Enable (ENB) */
	pinMode(10, OUTPUT);	/* Right Motor Enable (ENA) */
	pinMode( 9, OUTPUT);	/* Left Motor +ve terminal (IN4) */
	pinMode( 8, OUTPUT);	/* Left Motor -ve terminal (IN3) */
	pinMode( 7, OUTPUT);	/* Right Motor -ve terminal (IN2) */
	pinMode( 6, OUTPUT);	/* Right Motor +ve terminal (IN1) */
}

void loop() {
	speed = analogRead(A1);
	if (speed > 550) {
		/**
		 * If the Joystick is in the Upper region
		 * Set the motors direction to move forward
		 * */
		digitalWrite(9, HIGH);
		digitalWrite(8, LOW);
		digitalWrite(7, LOW);
		digitalWrite(6, HIGH);
		speed = speed - 512;
	} else if (speed < 470) {
		/**
		 * If the Joystick is in the Lower region
		 * Set the motors direction to move backward
		 * */
		digitalWrite(9, LOW);
		digitalWrite(8, HIGH);
		digitalWrite(7, HIGH);
		digitalWrite(6, LOW);
		speed = 512 - speed;
	} else {
		/**
		 * If the Joystick is in the Vertically Centered
		 * Turn off the motors
		 * */
		digitalWrite(9, LOW);
		digitalWrite(8, LOW);
		digitalWrite(7, LOW);
		digitalWrite(6, LOW);
		speed = 0;
	}
	/* Map the speed to the output range */
	speed = map(speed, 0, 512, 0, 255);

	direction = analogRead(A0);
	if (direction > 550 || direction < 470) {
		/**
		 * If the Joystick is either in the left or the right regions
		 * motors will get different speeds according to the joystick position
		 * */
		speedA = 1023 - direction;	/* Joystick position relative to the right side */
		speedB = direction;			/* Joystick position relative to the left side */
		/**
		 * The one with the maximum joystick position will get full speed
		 * the other one will get a fraction of full speed equivalent to the ratio
		 * between its joystick position and the maximum joystick position
		 * */
		max_speed = max(speedA, speedB);
		speedA = map(speedA, 0, max_speed, 0, speed);
		speedB = map(speedB, 0, max_speed, 0, speed);
	} else {
		/**
		 * If Joystick is Horizontally Centered
		 * both motors get the full speed
		 * */
		speedA = speed;
		speedB = speed;
	}
	/* Writing the output */
	analogWrite(10, speedA);
	analogWrite(11, speedB);

	/* To reduce simulation computations */
	// delay(20);
}
