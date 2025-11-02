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

class JointLocation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gripper = null;
      this.arm_1 = null;
      this.arm_2 = null;
      this.arm_3 = null;
      this.arm_4 = null;
      this.arm_5 = null;
      this.arm_6 = null;
    }
    else {
      if (initObj.hasOwnProperty('gripper')) {
        this.gripper = initObj.gripper
      }
      else {
        this.gripper = 0.0;
      }
      if (initObj.hasOwnProperty('arm_1')) {
        this.arm_1 = initObj.arm_1
      }
      else {
        this.arm_1 = 0.0;
      }
      if (initObj.hasOwnProperty('arm_2')) {
        this.arm_2 = initObj.arm_2
      }
      else {
        this.arm_2 = 0.0;
      }
      if (initObj.hasOwnProperty('arm_3')) {
        this.arm_3 = initObj.arm_3
      }
      else {
        this.arm_3 = 0.0;
      }
      if (initObj.hasOwnProperty('arm_4')) {
        this.arm_4 = initObj.arm_4
      }
      else {
        this.arm_4 = 0.0;
      }
      if (initObj.hasOwnProperty('arm_5')) {
        this.arm_5 = initObj.arm_5
      }
      else {
        this.arm_5 = 0.0;
      }
      if (initObj.hasOwnProperty('arm_6')) {
        this.arm_6 = initObj.arm_6
      }
      else {
        this.arm_6 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointLocation
    // Serialize message field [gripper]
    bufferOffset = _serializer.float64(obj.gripper, buffer, bufferOffset);
    // Serialize message field [arm_1]
    bufferOffset = _serializer.float64(obj.arm_1, buffer, bufferOffset);
    // Serialize message field [arm_2]
    bufferOffset = _serializer.float64(obj.arm_2, buffer, bufferOffset);
    // Serialize message field [arm_3]
    bufferOffset = _serializer.float64(obj.arm_3, buffer, bufferOffset);
    // Serialize message field [arm_4]
    bufferOffset = _serializer.float64(obj.arm_4, buffer, bufferOffset);
    // Serialize message field [arm_5]
    bufferOffset = _serializer.float64(obj.arm_5, buffer, bufferOffset);
    // Serialize message field [arm_6]
    bufferOffset = _serializer.float64(obj.arm_6, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointLocation
    let len;
    let data = new JointLocation(null);
    // Deserialize message field [gripper]
    data.gripper = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [arm_1]
    data.arm_1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [arm_2]
    data.arm_2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [arm_3]
    data.arm_3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [arm_4]
    data.arm_4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [arm_5]
    data.arm_5 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [arm_6]
    data.arm_6 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'xarm_driver/JointLocation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bfa33ef15f3f0cc9e792db497bca66aa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 gripper
    float64 arm_1
    float64 arm_2
    float64 arm_3
    float64 arm_4
    float64 arm_5
    float64 arm_6
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointLocation(null);
    if (msg.gripper !== undefined) {
      resolved.gripper = msg.gripper;
    }
    else {
      resolved.gripper = 0.0
    }

    if (msg.arm_1 !== undefined) {
      resolved.arm_1 = msg.arm_1;
    }
    else {
      resolved.arm_1 = 0.0
    }

    if (msg.arm_2 !== undefined) {
      resolved.arm_2 = msg.arm_2;
    }
    else {
      resolved.arm_2 = 0.0
    }

    if (msg.arm_3 !== undefined) {
      resolved.arm_3 = msg.arm_3;
    }
    else {
      resolved.arm_3 = 0.0
    }

    if (msg.arm_4 !== undefined) {
      resolved.arm_4 = msg.arm_4;
    }
    else {
      resolved.arm_4 = 0.0
    }

    if (msg.arm_5 !== undefined) {
      resolved.arm_5 = msg.arm_5;
    }
    else {
      resolved.arm_5 = 0.0
    }

    if (msg.arm_6 !== undefined) {
      resolved.arm_6 = msg.arm_6;
    }
    else {
      resolved.arm_6 = 0.0
    }

    return resolved;
    }
};

module.exports = JointLocation;
