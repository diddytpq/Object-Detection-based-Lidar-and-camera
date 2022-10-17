# VESCuino CDI CUSTOM_APP RX Messages

Header  header
int32 can_devs_num

# Common Data
int32[] can_id
float64[] voltage_input        # input voltage (volt)
float64[] temperature_pcb      # temperature of printed circuit board (degrees Celsius)
float64[] temperature_motor    # temperature of printed circuit board (degrees Celsius)
float64[] current_motor        # motor current (ampere)
float64[] current_input        # input current (ampere)
float64[] duty_cycle           # duty cycle (0 to 1)
float64[] watt_hours           # watt hours
float64[] watt_hours_charged   # watt hours charged
float64[] accum_deg_now        # accumulated degree now
float64[] diff_deg_now	       # degreee difference in 0.1ms(10kHz)