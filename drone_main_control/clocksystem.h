unsigned long period = 5;
unsigned long last_time ,Mem_last_time ;
bool CLK_10ms, CLK_10ms_Positive;
bool CLK_500ms;
bool CLK_500ms_Positive, CLK_500ms_Negative;
bool CLK_100ms_Positive,CLK_100ms;

int Count_100ms, Count_500ms, Count_1000ms,Count_50ms;

bool CLK_50ms;

void timesys() {

  CLK_10ms_Positive = 0;
  CLK_500ms_Positive = 0;
  CLK_500ms_Negative = 0;
  CLK_100ms_Positive = 0;

  Mem_last_time = millis();
  if (Mem_last_time - last_time >= period && Mem_last_time > last_time) {
    last_time = Mem_last_time;
    CLK_10ms = !CLK_10ms;
    CLK_10ms_Positive = CLK_10ms;
    Count_50ms++;
    Count_500ms++;
    Count_100ms++;
    Count_1000ms++;
    if (Count_100ms >= 20) {
      CLK_100ms =! CLK_100ms;
      CLK_100ms_Positive = 1;
      Count_100ms = 0;
    }

    if (Count_50ms >= 10) {
      Count_50ms = 0;
      CLK_50ms = !CLK_50ms;
    }

    if (Count_500ms >= 100) {
      Count_500ms = 0;
      CLK_500ms = !CLK_500ms;
      CLK_500ms_Positive = CLK_500ms;
      CLK_500ms_Positive = !CLK_500ms;
    }
  }
}