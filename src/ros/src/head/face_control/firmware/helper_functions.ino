// Helper functions for the py_eyes sketch

// top lid down animation routine
// Right now this test always passes and it is up to the user to observe the output
// to determine if overything went OK

int frame_4(const uint8_t animation_name[][8]){	
	for (uint8_t i=0; i<4; ++i){
		matrix[EYES].clear();
		matrix[EYES].drawBitmap (0, 0, animation_name[i], 8, 8, LED_ON);
		matrix[EYES].writeDisplay();
		delay(20);
	}
	// hold
	delay(1000);
	for (uint8_t j=4; j>0; --j){
		matrix[EYES].clear();
		matrix[EYES].drawBitmap (0, 0, animation_name[j], 8, 8, LED_ON);
		matrix[EYES].writeDisplay();
		delay(20);
	}
	return 0;
}






// Power on self test
//
// Right now this test always passes and it is up to the user
// to observe the output to determine if overything went OK
int post(Adafruit_8x8matrix matrix) {
  
  matrix.setTextSize(1);
  matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  matrix.setTextColor(LED_ON);
  for (int8_t x=0; x>=-36; x--) {
    digitalWrite(13,HIGH);
    matrix.clear();
    matrix.setCursor(x,0);
    matrix.print("Hello");
    matrix.writeDisplay();
    digitalWrite(13,LOW);
    delay(100); 
  }
  return(0); 
}

// function to read from serial
int readline(int readch, char *buffer, int len){
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len-1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}

  
