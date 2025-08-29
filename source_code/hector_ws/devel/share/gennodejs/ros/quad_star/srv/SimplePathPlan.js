// Auto-generated. Do not edit!

// (in-package quad_star.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SimplePathPlanRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_x = null;
      this.start_y = null;
      this.goal_x = null;
      this.goal_y = null;
    }
    else {
      if (initObj.hasOwnProperty('start_x')) {
        this.start_x = initObj.start_x
      }
      else {
        this.start_x = 0.0;
      }
      if (initObj.hasOwnProperty('start_y')) {
        this.start_y = initObj.start_y
      }
      else {
        this.start_y = 0.0;
      }
      if (initObj.hasOwnProperty('goal_x')) {
        this.goal_x = initObj.goal_x
      }
      else {
        this.goal_x = 0.0;
      }
      if (initObj.hasOwnProperty('goal_y')) {
        this.goal_y = initObj.goal_y
      }
      else {
        this.goal_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SimplePathPlanRequest
    // Serialize message field [start_x]
    bufferOffset = _serializer.float32(obj.start_x, buffer, bufferOffset);
    // Serialize message field [start_y]
    bufferOffset = _serializer.float32(obj.start_y, buffer, bufferOffset);
    // Serialize message field [goal_x]
    bufferOffset = _serializer.float32(obj.goal_x, buffer, bufferOffset);
    // Serialize message field [goal_y]
    bufferOffset = _serializer.float32(obj.goal_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SimplePathPlanRequest
    let len;
    let data = new SimplePathPlanRequest(null);
    // Deserialize message field [start_x]
    data.start_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [start_y]
    data.start_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [goal_x]
    data.goal_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [goal_y]
    data.goal_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'quad_star/SimplePathPlanRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '480eff7ac3d8ac00f5d0f116d00eccc9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 请求消息
    float32 start_x
    float32 start_y
    float32 goal_x
    float32 goal_y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SimplePathPlanRequest(null);
    if (msg.start_x !== undefined) {
      resolved.start_x = msg.start_x;
    }
    else {
      resolved.start_x = 0.0
    }

    if (msg.start_y !== undefined) {
      resolved.start_y = msg.start_y;
    }
    else {
      resolved.start_y = 0.0
    }

    if (msg.goal_x !== undefined) {
      resolved.goal_x = msg.goal_x;
    }
    else {
      resolved.goal_x = 0.0
    }

    if (msg.goal_y !== undefined) {
      resolved.goal_y = msg.goal_y;
    }
    else {
      resolved.goal_y = 0.0
    }

    return resolved;
    }
};

class SimplePathPlanResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SimplePathPlanResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SimplePathPlanResponse
    let len;
    let data = new SimplePathPlanResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'quad_star/SimplePathPlanResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 响应消息
    bool success
    string message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SimplePathPlanResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SimplePathPlanRequest,
  Response: SimplePathPlanResponse,
  md5sum() { return 'cc0fa8f370b2645f29bc836e78ed2538'; },
  datatype() { return 'quad_star/SimplePathPlan'; }
};
