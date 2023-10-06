// Tempest's buttons library

class TButton {

  public:
  // Setup function
  TButton(String name, int count) {
    buttonName = name;
    debounceCount = count;
    debounce = 0;
    state = false;
  }

  void update(bool inputState) {
    if (inputState != state) {
      debounce += 1;
    } else {
      debounce = 0;
    }

    if (debounce == debounceCount) {
      state = inputState;
      debounce = 0;
      Serial.print("#" + buttonName);
      if (state) {Serial.println("1");} else {Serial.println("0");}
    }

  }

  private:
  String buttonName;
  bool state;
  int debounce;
  int debounceCount;

};