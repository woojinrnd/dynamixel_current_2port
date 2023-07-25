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

class Turn_AngleRequest {
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
    // Serializes a message object of type Turn_AngleRequest
    // Serialize message field [finish]
    bufferOffset = _serializer.bool(obj.finish, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Turn_AngleRequest
    let len;
    let data = new Turn_AngleRequest(null);
    // Deserialize message field [finish]
    data.finish = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamixel_current_2port/Turn_AngleRequest';
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
    const resolved = new Turn_AngleRequest(null);
    if (msg.finish !== undefined) {
      resolved.finish = msg.finish;
    }
    else {
      resolved.finish = false
    }

    return resolved;
    }
};

class Turn_AngleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.turn_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('turn_angle')) {
        this.turn_angle = initObj.turn_angle
      }
      else {
        this.turn_angle = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Turn_AngleResponse
    // Serialize message field [turn_angle]
    bufferOffset = _serializer.int8(obj.turn_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Turn_AngleResponse
    let len;
    let data = new Turn_AngleResponse(null);
    // Deserialize message field [turn_angle]
    data.turn_angle = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamixel_current_2port/Turn_AngleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3ffb826ed844fc50fc9b5acac9831493';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 turn_angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Turn_AngleResponse(null);
    if (msg.turn_angle !== undefined) {
      resolved.turn_angle = msg.turn_angle;
    }
    else {
      resolved.turn_angle = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: Turn_AngleRequest,
  Response: Turn_AngleResponse,
  md5sum() { return '6bcc5ac74e5ed33763d25bed29ca1508'; },
  datatype() { return 'dynamixel_current_2port/Turn_Angle'; }
};
