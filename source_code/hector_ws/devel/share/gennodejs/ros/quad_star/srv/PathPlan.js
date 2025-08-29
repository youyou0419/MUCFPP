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

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class PathPlanRequest {
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
    // Serializes a message object of type PathPlanRequest
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
    //deserializes a message object of type PathPlanRequest
    let len;
    let data = new PathPlanRequest(null);
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
    return 'quad_star/PathPlanRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '480eff7ac3d8ac00f5d0f116d00eccc9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new PathPlanRequest(null);
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

class PathPlanResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
      this.path = null;
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
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathPlanResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [path]
    // Serialize the length for message field [path]
    bufferOffset = _serializer.uint32(obj.path.length, buffer, bufferOffset);
    obj.path.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathPlanResponse
    let len;
    let data = new PathPlanResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [path]
    // Deserialize array length for message field [path]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.path = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.path[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    length += 24 * object.path.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'quad_star/PathPlanResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f7dcb9d698f920b09092283d36a2c9f3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string message
    geometry_msgs/Point[] path
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PathPlanResponse(null);
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

    if (msg.path !== undefined) {
      resolved.path = new Array(msg.path.length);
      for (let i = 0; i < resolved.path.length; ++i) {
        resolved.path[i] = geometry_msgs.msg.Point.Resolve(msg.path[i]);
      }
    }
    else {
      resolved.path = []
    }

    return resolved;
    }
};

module.exports = {
  Request: PathPlanRequest,
  Response: PathPlanResponse,
  md5sum() { return 'c9c9ee362512ec5be6042c67e74ac40f'; },
  datatype() { return 'quad_star/PathPlan'; }
};
