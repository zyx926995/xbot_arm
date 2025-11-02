// Auto-generated. Do not edit!

// (in-package xarm_pick_place.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ArmControlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joints_values = null;
    }
    else {
      if (initObj.hasOwnProperty('joints_values')) {
        this.joints_values = initObj.joints_values
      }
      else {
        this.joints_values = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmControlRequest
    // Check that the constant length array field [joints_values] has the right length
    if (obj.joints_values.length !== 6) {
      throw new Error('Unable to serialize array field joints_values - length must be 6')
    }
    // Serialize message field [joints_values]
    bufferOffset = _arraySerializer.float32(obj.joints_values, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmControlRequest
    let len;
    let data = new ArmControlRequest(null);
    // Deserialize message field [joints_values]
    data.joints_values = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_pick_place/ArmControlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '31d35b8ea9c05e2d738767347cc083d4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[6] joints_values
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmControlRequest(null);
    if (msg.joints_values !== undefined) {
      resolved.joints_values = msg.joints_values;
    }
    else {
      resolved.joints_values = new Array(6).fill(0)
    }

    return resolved;
    }
};

class ArmControlResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmControlResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmControlResponse
    let len;
    let data = new ArmControlResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_pick_place/ArmControlResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmControlResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: ArmControlRequest,
  Response: ArmControlResponse,
  md5sum() { return 'bdae41a092f6ee8c19a55dc64ce0bff1'; },
  datatype() { return 'xarm_pick_place/ArmControl'; }
};
