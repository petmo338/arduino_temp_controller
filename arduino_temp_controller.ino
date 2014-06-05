#define SERIAL_INPUT_LENGHT 4
#define NR_OF_MEAN_SAMPLES 5
#define USE_PID_INTEGRAL_THRESHOLD 12
#define MIN_INTEGRAL -50
#define MAX_INTEGRAL 30
#define DEBUG 1
#define PID_DEBUG 0
#define TEMP_DEBUG 0
#define SERIAL_DEBUG 0
#define RESISTANCE_OFFSET (82)
#define PT100_TABLE_STEP_DEG 30.0
#define MIN_TEMP (-10)
#define MAX_TEMP 680
#define PID_INTERVAL_MS 50
#define DT ((float)PID_INTERVAL_MS/1000.0)
#define HEATER_CURVE_STEP 50
enum STATE {
    TEMP_MEASURE,
    CHECK_SERIAL,
    PID_CALC,
    TIMING_SYNC,
    PWM_SET,
};
float const Pt100[] = {    96.09, 107.79, 119.40, 130.90, 142.29, 153.58,
                          164.77, 175.86, 186.84, 197.71, 208.48, 219.15,
                          229.72, 240.18, 250.35, 260.78, 270.93, 280.98,
                          290.92, 300.75, 310.49, 320.12, 329.64, 339.06 };
uint8_t const one_heater_curve[] = {0, 1, 2, 4, 7, 10, 13, 17, 22, 28, 33, 40, 48, 56, 63};

STATE State;
uint8_t TEMP_PIN = 15;
uint8_t PWM_PIN = 2;
uint64_t next_loop_iteration = 0;
int32_t current_temp = 0;
int32_t desired_temp = 0;
uint64_t current_time;
uint8_t* selected_heater_curve = NULL; 

float volt_res_slope = 0.016145;
float PID_integral = 0;
float previous_error = 0;
float Kp = 0.6;
float Ki = 0.25;
float Kd = 0.0;
uint8_t pwm_output = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(5);
  analogReference(DEFAULT);
  for (int i = 0; i < 5; i++)
  {
    analogRead(TEMP_PIN);
  }
  analogWrite(PWM_PIN, pwm_output);
  State = TEMP_MEASURE;
  selected_heater_curve = const_cast<uint8_t*>(one_heater_curve);
}

void loop() {
  // put your main code here, to run repeatedly:

  switch (State)
  {
    case TEMP_MEASURE:
      GetTemp();
      State = CHECK_SERIAL;
      break;
    case CHECK_SERIAL:
      CheckSerial();
      State = PID_CALC;
    break;
    case PID_CALC:
      PID_Calc();
      State = TIMING_SYNC;
    break;
    case TIMING_SYNC:
      current_time = millis();
      if (current_time > next_loop_iteration)
      {
        State = PWM_SET;
        next_loop_iteration =  current_time + PID_INTERVAL_MS;
/*        if (DEBUG != 0)
        {
          Serial.print("Received temp:");
          Serial.println(desired_temp);
        }*/

      }
      break;
    case PWM_SET:
      analogWrite(PWM_PIN, pwm_output);
      State = TEMP_MEASURE;
    break;  
  }
}

void PID_Calc()
{
  float error = desired_temp - current_temp;
//  float dt = (int)PID_INTERVAL_MS / 1000;
//  float dt = 0.1;
  PID_integral += error * DT;
  if (abs(error) > USE_PID_INTEGRAL_THRESHOLD)
  {
    PID_integral = 0;
  }
  else if (PID_integral < MIN_INTEGRAL)
  {
    PID_integral = MIN_INTEGRAL;
  }
  else if (PID_integral > MAX_INTEGRAL)
  {
    PID_integral = MAX_INTEGRAL;
  }
  float derivative = (error - previous_error) / DT;
  float out = (Kp * error + Ki * PID_integral + Kd * derivative);

  uint8_t baseline = selected_heater_curve[desired_temp / HEATER_CURVE_STEP];
  float slope = (float)(selected_heater_curve[(desired_temp / HEATER_CURVE_STEP) + 1] - baseline) / (float)HEATER_CURVE_STEP;
  baseline += (desired_temp % HEATER_CURVE_STEP) * slope;
  out += baseline;

  if (out > 255)
  {
    pwm_output = 255;
  }
  else if (out < 0)
  {
    pwm_output = 0;
  }
  else
  {
    pwm_output = out;
  }
  previous_error = error;
  if (PID_DEBUG != 0)
  {
    Serial.print(" des_temp, ");
    Serial.print(desired_temp);
    Serial.print(" curr_temp, ");
    Serial.print(current_temp);
    Serial.print(" error, ");
    Serial.print(error);
    Serial.print(" PID_integral, ");
    Serial.print(PID_integral);
    Serial.print(" dev, ");
    Serial.print(derivative);
    Serial.print(" output, ");
    Serial.print(pwm_output);
    Serial.print(" baseline, ");
    Serial.print(baseline);
    Serial.print(" prev_error, ");
    Serial.println(previous_error); 
  }
}

void CheckSerial()
{
  int temperature = MIN_TEMP;
  if (Serial.available() > 0) {
    int c = Serial.peek();
    //Serial.print(c);
    //Serial.print("vafan");
    if (c == 'C')
    {
      c = Serial.read();
      Serial.println("OK");
    }
    else if (c == 'S')
    {
      Serial.read();
      temperature = Serial.parseInt();
    }
    else if (c == 'T')
    {
      Serial.read();      
      Serial.println(current_temp);
    }
    else
    {
      while (Serial.available() > 0)
      {
        Serial.read();
      }
    }
  }
  if ((temperature > MIN_TEMP) && (temperature < MAX_TEMP))
  {
    desired_temp = temperature;
  }
  if (SERIAL_DEBUG != 0)
  {
    Serial.print("Received temp:");
    Serial.println(desired_temp);
  }
}

void GetTemp()
{
  uint32_t mean_input = 0;
  bool not_found = true;
  int i;
  for (int k = 0; k < NR_OF_MEAN_SAMPLES; k++)
  {
    mean_input += analogRead(TEMP_PIN);
  }
  mean_input /= NR_OF_MEAN_SAMPLES;
  float mean_voltage = 5 * ((float)mean_input / 1024.0);
  float resistance = RESISTANCE_OFFSET + (float)mean_voltage / volt_res_slope;
  for (i = 0; i < (sizeof(Pt100) / sizeof(float)); i++)
  {
    if (resistance < Pt100[i])
    {
      current_temp = PT100_TABLE_STEP_DEG * ((resistance - Pt100[i - 1]) / (Pt100[i] - Pt100[i - 1]) + (i - 1)) + MIN_TEMP;
      not_found = false;
      break;
    }     
  }
  if (not_found == true)
  {
    current_temp = MAX_TEMP;
    Serial.print("Resistance too high");
  }
  if (TEMP_DEBUG != 0)
  {
    Serial.print("Measured volt:");
    Serial.print(mean_voltage);
    Serial.print(", calc res::");
    Serial.print(resistance);
    Serial.print(", Calc temp:");
    Serial.println(current_temp);
  }
}
