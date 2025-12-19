ENTITY_MAP = {
  # FC04 sensors
  "sensor.cv_forward_temp": {
    "source": ("input_registers", "CV Forward"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature"
    }
  },
  "sensor.cv_return_temp": {
    "source": ("input_registers", "CV Return"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature"
    }
  },
  "sensor.storage_tank_vv_temp": {
    "source": ("input_registers", "Storage tank VV"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature"
    }
  },
  "sensor.storage_tank_cv_temp": {
    "source": ("input_registers", "Storage tank CV"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature"
    }
  },
  "sensor.evaporator_temp": {
    "source": ("input_registers", "Evaporator"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature"
    }
  },
  "sensor.outdoor_temp": {
    "source": ("input_registers", "Outdoor"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature"
    }
  },
  "sensor.compressor_hp": {
    "source": ("input_registers", "Compressor HP"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature"
    }
  },
  "sensor.compressor_lp": {
    "source": ("input_registers", "Compressor LP"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature"
    }
  },
  "sensor.em23_power": {
    "source": ("input_registers", "em23_power"), 
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "kW",
      "device_class": "power"
    }
  },
  "sensor.em23_energy": {
    "source": ("input_registers", "em23_energy"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "kWh",
      "device_class": "energy",
      "state_class": "total_increasing"
    }
  },
  # FC01 coils
  "binary_sensor.soft_starter_compressor": {
    "source": ("coils", "Soft starter Compressor"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.three_way_shunt_vv_open_close": {
    "source": ("coils", "3-Way shunt VV open/close"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.start_stop_expansion_valve": {
    "source": ("coils", "Start/stop expansion valve"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.heating_element": {
    "source": ("coils", "Heating element"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.circ_pump_warm_side": {
    "source": ("coils", "Circ. pump warm side"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.el_tracing_cv_drain": {
    "source": ("coils", "El-tracing CV/drain"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.four_way_valve_defrost": {
    "source": ("coils", "4-way valve defrost"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.liquid_injection_solenoid_valve": {
    "source": ("coils", "Liquid injection solenoid valve"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.three_way_shunt_cv_open": {
    "source": ("coils", "3-way shunt CV open"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.three_way_shunt_cv_close": {
    "source": ("coils", "3-way shunt CV close"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.circ_pump_cv": {
    "source": ("coils", "Circ. pump CV"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "running"
    }
  },
  "binary_sensor.sum_alarm_failure": {
    "source": ("coils", "Sum alarm failure"),
    "domain": "binary_sensor",
    "attributes": {
      "device_class": "problem"
    }
  },
  # FC06 selects
  "select.cv_mode": {
    "source": ("write_registers", "cv_mode"),
    "domain": "select",
    "command_topic": "dvi/command/cvstate",
    "attributes": {
      "options": ["Off", "On"]
    },
    "mapping": {0: "Off", 1: "On"}
  },
  "select.cv_night": {
    "source": ("write_registers", "cv_night"),
    "domain": "select",
    "command_topic": "dvi/command/cvnight",
    "attributes": {
      "options": ["Timer", "Constant day", "Constant night"]
    },
    "mapping": {0: "Timer", 1: "Constant day", 2: "Constant night"}
  },
  "select.vv_mode": {
    "source": ("write_registers", "vv_mode"),
    "domain": "select",
    "command_topic": "dvi/command/vvstate",
    "attributes": {
      "options": ["Off", "On", "Timer"]
    },
    "mapping": {0: "Off", 1: "On", 2: "Timer"}
  },
  "select.vv_schedule": {
    "source": ("write_registers", "vv_schedule"),
    "domain": "select",
    "command_topic": "dvi/command/vvschedule",
    "attributes": {
      "options": ["Timer", "Constant on", "Constant off"]
    },
    "mapping": {0: "Timer", 1: "Constant on", 2: "Constant off"}
  },
  "select.aux_heating": {
    "source": ("write_registers", "aux_heating"),
    "domain": "select",
    "command_topic": "dvi/command/tvstate",
    "attributes": {
      "options": ["Off", "Automatic", "Backup operation"]
    },
    "mapping": {0: "Off", 1: "Automatic", 2: "Backup operation"}
  },
  "select.central_heating_config": {
    "source": ("write_registers", "central_heating_config"),
    "domain": "select",
    "command_topic": "dvi/command/centralheatingconfig",
    "attributes": {
      "options": ["Under floor heating w/o shunt", "Under floor heating w. shunt", "Radiator and mixed systems"]
    },
    "mapping": {0: "Under floor heating w/o shunt", 1: "Under floor heating w. shunt", 2: "Radiator and mixed systems"}
  },
  # FC06 numbers
  "number.cv_curve": {
    "source": ("write_registers", "cv_curve"),
    "domain": "number",
    "command_topic": "dvi/command/cvcurve",
    "attributes": {
      "min": 1,
      "max": 20,
      "step": 1
    }
  },
  "number.curve_set_minus12": {
    "source": ("write_registers", "curve_set_-12_read"),
    "domain": "number",
    "command_topic": "dvi/command/curveset-12",
    "attributes": {
      "min": 20,
      "max": 60,
      "step": 1,
      "unit_of_measurement": "°C"
    }
  },
  "number.curve_set_plus12": {
    "source": ("write_registers", "curve_set_12_read"),
    "domain": "number",
    "command_topic": "dvi/command/curveset12",
    "attributes": {
      "min": 20,
      "max": 60,
      "step": 1,
      "unit_of_measurement": "°C"
    }
  },
  "number.vv_setpoint": {
    "source": ("write_registers", "vv_setpoint"),
    "domain": "number",
    "command_topic": "dvi/command/vvsetpoint",
    "attributes": {
      "min": 10,
      "max": 60,
      "step": 1,
      "unit_of_measurement": "°C"
    }
  },
  "number.cv_max": {
    "source": ("write_registers", "cv_max"),
    "domain": "number",
    "command_topic": "dvi/command/cvmax",
    "attributes": {
      "min": 20,
      "max": 55,
      "step": 1,
      "unit_of_measurement": "°C"
    }
  },
  "number.cv_min": {
    "source": ("write_registers", "cv_min"),
    "domain": "number",
    "command_topic": "dvi/command/cvmin",
    "attributes": {
      "min": 10,
      "max": 45,
      "step": 1,
      "unit_of_measurement": "°C"
    }
  },
  # FC06 sensors
  "sensor.outdoor_cal": {
    "source": ("write_registers", "outdoor_cal"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature",
      "state_class": "measurement"
    }
  },
  "sensor.curve_temp": {
    "source": ("write_registers", "curve_temp"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature",
      "state_class": "measurement"
    }
  },
  "sensor.cv_setpoint": {
    "source": ("write_registers", "cv_setpoint"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "°C",
      "device_class": "temperature",
      "state_class": "measurement"
    }
  },
  "sensor.comp_hours": {
    "source": ("write_registers", "comp_hours"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "h",
      "device_class": "duration",
      "state_class": "total_increasing"
    }
  },
  "sensor.vv_hours": {
    "source": ("write_registers", "vv_hours"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "h",
      "device_class": "duration",
      "state_class": "total_increasing"
    }
  },
  "sensor.heating_hours": {
    "source": ("write_registers", "heating_hours"),
    "domain": "sensor",
    "attributes": {
      "unit_of_measurement": "h",
      "device_class": "duration",
      "state_class": "total_increasing"
    }
  }
}

