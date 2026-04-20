#define PIN_STA PD_2
#define PIN_IDE A11
#define PIN_VSI A9

#define PIN_RGB_R PF_1
#define PIN_RGB_G PF_3
#define PIN_RGB_B PF_2

#define VIDE_MPG401 1.516
#define VIDE_DISCONN 3.3
#define EPS_VIDE 0.2

#define UN_VDIV(NAME) ((VDIV_R1_##NAME / VDIV_R2_##NAME) / VDIV_R2_##NAME)
#define VDIV_R1_VSI ((double)5.1)
#define VDIV_R2_VSI ((double)2)

#define BITS_ADC 12
#define BMAX_ADC (1 << BITS_ADC)
#define AVOLTS 3.3

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_STA, INPUT);
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  Serial.begin(9600);
}

double analogReadV(int pin) {
  return (((double)analogRead(pin)) / BMAX_ADC) * AVOLTS;
}

enum SenType { TYPE_UNKNOWNSEN, TYPE_MPG401 };

enum SenErr {
  SEN_OK,
  SEN_ERR_DISCONN,
  SEN_ERR_NOSUP,
  SEN_ERR_PIRANIFAIL,
  SEN_ERR_OVERRANGE,
  SEN_ERR_UNDERRANGE,
  SEN_ERR_UNKNOWNSEN
};

struct SenData {
  enum SenErr err;
  bool cc_mode;
  enum SenType type;
  double p;
};

double convertPressure(double vvsi) {
  double p_exp = (vvsi - 8.6) / 0.6;
  double p = pow(10, p_exp) * 750;
  return p;
}

void readSensor(struct SenData* sen) {
  sen->type = TYPE_UNKNOWNSEN;
  sen->p = 0;
  sen->err = SEN_OK;
  sen->cc_mode = digitalRead(PIN_STA) == HIGH;
  double vide = analogReadV(PIN_IDE);
  if (abs(vide - VIDE_MPG401) < EPS_VIDE) sen->type = TYPE_MPG401;
  if (abs(vide - VIDE_DISCONN) < EPS_VIDE) {
    sen->err = SEN_ERR_DISCONN;
  } else {
      if (sen->type == TYPE_UNKNOWNSEN) sen->err = SEN_ERR_UNKNOWNSEN;
  }
  double vvsi = analogReadV(PIN_VSI) * UN_VDIV(VSI);
  if (vvsi < 0.7) {
    sen->err = SEN_ERR_NOSUP;
  } else if (vvsi > 9.3) {
    sen->err = SEN_ERR_PIRANIFAIL;
  } else if (vvsi > 8.8) {
    sen->err = SEN_ERR_OVERRANGE;
  } else if (vvsi < 1.82) {
    sen->err = SEN_ERR_UNDERRANGE;
  } else {
    sen->p = convertPressure(vvsi);
  }
}

struct SenData sens;

void loop() {
  readSensor(&sens);
  Serial.println(sens.err);
  Serial.println(sens.p);
  Serial.println(analogReadV(PIN_VSI));
  if (!sens.err) {
    digitalWrite(PIN_RGB_R, LOW);
    if ((abs(sens.p) - 760) < 30) {
      digitalWrite(PIN_RGB_G, HIGH);
      digitalWrite(PIN_RGB_B, LOW);
    } else {
      digitalWrite(PIN_RGB_G, LOW);
      digitalWrite(PIN_RGB_B, HIGH);  
    }
  } else {
    digitalWrite(PIN_RGB_R, HIGH);
    digitalWrite(PIN_RGB_G, LOW);
    digitalWrite(PIN_RGB_B, LOW);
  }
  delay(500);
}
