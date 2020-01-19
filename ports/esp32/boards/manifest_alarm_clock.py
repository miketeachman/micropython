include('manifest.py')

freeze('$(MPY_LIB_DIR)/logging', 'logging.py')
freeze('C:/Users/miket/Documents/micropython_projects/micropython-async', ('uasyncio/__init__.py', 'uasyncio/core.py', 'aswitch.py'))
freeze('C:/Users/miket/Documents/micropython_projects/Adafruit-uRTC', ('urtc.py'))
freeze('C:/Users/miket/Documents/micropython_projects/micropython-rotary', ('rotary.py', 'rotary_irq_esp.py'))
#freeze('C:/Users/miket/Documents/micropython_projects/alarmclock', ('alarmclock.py'))