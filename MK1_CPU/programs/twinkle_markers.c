// Twinkle with progress markers
void phrase1() {
  tone(262, 400); silence(100);
  tone(262, 400); silence(100);
  tone(392, 400); silence(100);
  tone(392, 400); silence(100);
}

void phrase2() {
  tone(440, 400); silence(100);
  tone(440, 400); silence(100);
  tone(392, 800); silence(200);
}

void phrase3() {
  tone(349, 400); silence(100);
  tone(349, 400); silence(100);
  tone(330, 400); silence(100);
  tone(330, 400); silence(100);
}

void phrase4() {
  tone(294, 400); silence(100);
  tone(294, 400); silence(100);
  tone(262, 800); silence(200);
}

void main() {
  i2c_init();
  delay_calibrate();
  out(1);     // init done
  phrase1();
  out(2);     // phrase1 done
  phrase2();
  out(3);     // phrase2 done
  phrase3();
  out(4);     // phrase3 done
  phrase4();
  out(5);     // phrase4 done
  phrase1();
  out(6);     // repeat phrase1 done
  phrase2();
  out(99);    // all done
  halt();
}
