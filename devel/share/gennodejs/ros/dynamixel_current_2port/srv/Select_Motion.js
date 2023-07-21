// Auto-generated. Do not edit!

// (in-package dynamixel_current_2port.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class Select_MotionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.finish = null;
    }
    else {
      if (initObj.hasOwnProperty('finish')) {
        this.finish = initObj.finish
      }
      else {
        this.finish = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Select_MotionRequest
    // Serialize message field [finish]
    bufferOffset = _serializer.bool(obj.finish, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Select_MotionRequest
    let len;
    let data = new Select_MotionRequest(null);
    // Deserialize message field [finish]
    data.finish = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamixel_current_2port/Select_MotionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '474a58dbb494a45bb1ca99544cd64e45';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool finish
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Select_MotionRequest(null);
    if (msg.finish !== undefined) {
      resolved.finish = msg.finish;
    }
    else {
      resolved.finish = false
    }

    return resolved;
    }
};

class Select_MotionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Select_Motion = null;
    }
    else {
      if (initObj.hasOwnProperty('Select_Motion')) {
        this.Select_Motion = initObj.Select_Motion
      }
      else {
        this.Select_Motion = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Select_MotionResponse
    // Serialize message field [Select_Motion]
    bufferOffset = _serializer.int8(obj.Select_Motion, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Select_MotionResponse
    let len;
    let data = new Select_MotionResponse(null);
    // Deserialize message field [Select_Motion]
    data.Select_Motion = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamixel_current_2port/Select_MotionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5aaa1cdf85b33c7430af49dec6f787c6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 Select_Motion
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Select_MotionResponse(null);
    if (msg.Select_Motion !== undefined) {
      resolved.Select_Motion = msg.Select_Motion;
    }
    else {
      resolved.Select_Motion = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: Select_MotionRequest,
  Response: Select_MotionResponse,
  md5sum() { return 'cc9905de9a643bc5d56477c60453b2c8'; },
  datatype() { return 'dynamixel_current_2port/Select_Motion'; }
};
