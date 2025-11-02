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

class RemoveObjectsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.objects_id = null;
    }
    else {
      if (initObj.hasOwnProperty('objects_id')) {
        this.objects_id = initObj.objects_id
      }
      else {
        this.objects_id = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RemoveObjectsRequest
    // Serialize message field [objects_id]
    bufferOffset = _arraySerializer.string(obj.objects_id, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RemoveObjectsRequest
    let len;
    let data = new RemoveObjectsRequest(null);
    // Deserialize message field [objects_id]
    data.objects_id = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.objects_id.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_pick_place/RemoveObjectsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '49713b5f379c9b78330f0563a9648359';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] objects_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RemoveObjectsRequest(null);
    if (msg.objects_id !== undefined) {
      resolved.objects_id = msg.objects_id;
    }
    else {
      resolved.objects_id = []
    }

    return resolved;
    }
};

class RemoveObjectsResponse {
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
    // Serializes a message object of type RemoveObjectsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RemoveObjectsResponse
    let len;
    let data = new RemoveObjectsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xarm_pick_place/RemoveObjectsResponse';
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
    const resolved = new RemoveObjectsResponse(null);
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
  Request: RemoveObjectsRequest,
  Response: RemoveObjectsResponse,
  md5sum() { return '1c917203483ea92cdcc10db8ea0c542a'; },
  datatype() { return 'xarm_pick_place/RemoveObjects'; }
};
