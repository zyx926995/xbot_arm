// Auto-generated. Do not edit!

// (in-package xarm_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.location = null;
      this.speed = null;
      this.load = null;
      this.voltage = null;
      this.temperature = null;
      this.current = null;
      this.error = null;
    }
    else {
      if (initObj.hasOwnProperty('location')) {
        this.location = initObj.location
      }
      else {
        this.location = [];
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = [];
      }
      if (initObj.hasOwnProperty('load')) {
        this.load = initObj.load
      }
      else {
        this.load = [];
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = [];
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = [];
      }
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = [];
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorStatus
    // Serialize message field [location]
    bufferOffset = _arraySerializer.float32(obj.location, buffer, bufferOffset, null);
    // Serialize message field [speed]
    bufferOffset = _arraySerializer.float32(obj.speed, buffer, bufferOffset, null);
    // Serialize message field [load]
    bufferOffset = _arraySerializer.float32(obj.load, buffer, bufferOffset, null);
    // Serialize message field [voltage]
    bufferOffset = _arraySerializer.float32(obj.voltage, buffer, bufferOffset, null);
    // Serialize message field [temperature]
    bufferOffset = _arraySerializer.float32(obj.temperature, buffer, bufferOffset, null);
    // Serialize message field [current]
    bufferOffset = _arraySerializer.float32(obj.current, buffer, bufferOffset, null);
    // Serialize message field [error]
    bufferOffset = _arraySerializer.uint8(obj.error, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorStatus
    let len;
    let data = new MotorStatus(null);
    // Deserialize message field [location]
    data.location = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [speed]
    data.speed = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [load]
    data.load = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [voltage]
    data.voltage = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [temperature]
    data.temperature = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [current]
    data.current = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [error]
    data.error = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.location.length;
    length += 4 * object.speed.length;
    length += 4 * object.load.length;
    length += 4 * object.voltage.length;
    length += 4 * object.temperature.length;
    length += 4 * object.current.length;
    length += object.error.length;
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xarm_driver/MotorStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b35e70483b9ebb379f12dfaee0874ca1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] location
    float32[] speed
    float32[] load
    float32[] voltage
    float32[] temperature
    float32[] current
    uint8[] error
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorStatus(null);
    if (msg.location !== undefined) {
      resolved.location = msg.location;
    }
    else {
      resolved.location = []
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = []
    }

    if (msg.load !== undefined) {
      resolved.load = msg.load;
    }
    else {
      resolved.load = []
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = []
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = []
    }

    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = []
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = []
    }

    return resolved;
    }
};

module.exports = MotorStatus;
