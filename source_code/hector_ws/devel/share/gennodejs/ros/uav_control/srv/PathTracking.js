// Auto-generated. Do not edit!

// (in-package uav_control.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let UAVPath = require('../msg/UAVPath.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class PathTrackingRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ldr_id = null;
      this.uav_ids = null;
      this.uav_paths = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ldr_id')) {
        this.ldr_id = initObj.ldr_id
      }
      else {
        this.ldr_id = 0;
      }
      if (initObj.hasOwnProperty('uav_ids')) {
        this.uav_ids = initObj.uav_ids
      }
      else {
        this.uav_ids = [];
      }
      if (initObj.hasOwnProperty('uav_paths')) {
        this.uav_paths = initObj.uav_paths
      }
      else {
        this.uav_paths = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathTrackingRequest
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ldr_id]
    bufferOffset = _serializer.int32(obj.ldr_id, buffer, bufferOffset);
    // Serialize message field [uav_ids]
    bufferOffset = _arraySerializer.string(obj.uav_ids, buffer, bufferOffset, null);
    // Serialize message field [uav_paths]
    // Serialize the length for message field [uav_paths]
    bufferOffset = _serializer.uint32(obj.uav_paths.length, buffer, bufferOffset);
    obj.uav_paths.forEach((val) => {
      bufferOffset = UAVPath.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathTrackingRequest
    let len;
    let data = new PathTrackingRequest(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ldr_id]
    data.ldr_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [uav_ids]
    data.uav_ids = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [uav_paths]
    // Deserialize array length for message field [uav_paths]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.uav_paths = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.uav_paths[i] = UAVPath.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.uav_ids.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.uav_paths.forEach((val) => {
      length += UAVPath.getMessageSize(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_control/PathTrackingRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '66074861de72e89556e85750791c9c7b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # PathTracking.srv
    # 路径跟踪服务定义
    
    # Request - 发送路径信息
    std_msgs/Header header
    int32 ldr_id
    string[] uav_ids
    uav_control/UAVPath[] uav_paths
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: uav_control/UAVPath
    # 单个无人机的路径信息
    # 文件路径: uav_control/msg/UAVPath.msg
    
    string uav_id
    geometry_msgs/Point[] path_points
    int32[] time_indices
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
    const resolved = new PathTrackingRequest(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ldr_id !== undefined) {
      resolved.ldr_id = msg.ldr_id;
    }
    else {
      resolved.ldr_id = 0
    }

    if (msg.uav_ids !== undefined) {
      resolved.uav_ids = msg.uav_ids;
    }
    else {
      resolved.uav_ids = []
    }

    if (msg.uav_paths !== undefined) {
      resolved.uav_paths = new Array(msg.uav_paths.length);
      for (let i = 0; i < resolved.uav_paths.length; ++i) {
        resolved.uav_paths[i] = UAVPath.Resolve(msg.uav_paths[i]);
      }
    }
    else {
      resolved.uav_paths = []
    }

    return resolved;
    }
};

class PathTrackingResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
      this.execution_time = null;
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
      if (initObj.hasOwnProperty('execution_time')) {
        this.execution_time = initObj.execution_time
      }
      else {
        this.execution_time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathTrackingResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [execution_time]
    bufferOffset = _serializer.float64(obj.execution_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathTrackingResponse
    let len;
    let data = new PathTrackingResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [execution_time]
    data.execution_time = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_control/PathTrackingResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8dc547cb16b95701f3b64b7c9a288908';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # Response - 返回执行结果
    bool success
    string message
    float64 execution_time
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PathTrackingResponse(null);
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

    if (msg.execution_time !== undefined) {
      resolved.execution_time = msg.execution_time;
    }
    else {
      resolved.execution_time = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: PathTrackingRequest,
  Response: PathTrackingResponse,
  md5sum() { return '0ff8a4c9efe935b06b349c3681633f16'; },
  datatype() { return 'uav_control/PathTracking'; }
};
