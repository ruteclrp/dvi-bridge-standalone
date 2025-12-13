ENTITY_MAP = {
  "sensor.cv_forward": {
    "source": ("input_registers", "CV Forward"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature"
    }
  },
  "number.cv_curve": {
    "source": ("write_registers", "cv_curve"),
    "domain": "number",
    "command_topic": "dvi/command/cvcurve",
    "attributes": {
      "min": 1,
      "max": 20,
      "step": 1
    }
  }
}

