User Guide:
- Use a timer or systick timer for this driver.
- If you use a timer, timer frequency is equire to your frequency options.(ref: CLK_TickFreq) 
- Add increment function at timer interrupt function (CLK_Inc()).

CLK_TickFreq:
- CLK_TICK_FREQ_10HZ		= 100U,
- CLK_TICK_FREQ_100HZ		= 10U,
- CLK_TICK_FREQ_1KHZ		= 1U,
