// Auto-generated. Do not edit!

// (in-package uav_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ResolutionStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ldr_id = null;
      this.uav_ids = null;
      this.in_resolution = null;
      this.aabb_min_x = null;
      this.aabb_max_x = null;
      this.aabb_min_y = null;
      this.aabb_max_y = null;
    }
    else {
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
      if (initObj.hasOwnProperty('in_resolution')) {
        this.in_resolution = initObj.in_resolution
      }
      else {
        this.in_resolution = false;
      }
      if (initObj.hasOwnProperty('aabb_min_x')) {
        this.aabb_min_x = initObj.aabb_min_x
      }
      else {
        this.aabb_min_x = 0.0;
      }
      if (initObj.hasOwnProperty('aabb_max_x')) {
        this.aabb_max_x = initObj.aabb_max_x
      }
      else {
        this.aabb_max_x = 0.0;
      }
      if (initObj.hasOwnProperty('aabb_min_y')) {
        this.aabb_min_y = initObj.aabb_min_y
      }
      else {
        this.aabb_min_y = 0.0;
      }
      if (initObj.hasOwnProperty('aabb_max_y')) {
        this.aabb_max_y = initObj.aabb_max_y
      }
      else {
        this.aabb_max_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResolutionStatus
    // Serialize message field [ldr_id]
    bufferOffset = _serializer.int32(obj.ldr_id, buffer, bufferOffset);
    // Serialize message field [uav_ids]
    bufferOffset = _arraySerializer.string(obj.uav_ids, buffer, bufferOffset, null);
    // Serialize message field [in_resolution]
    bufferOffset = _serializer.bool(obj.in_resolution, buffer, bufferOffset);
    // Serialize message field [aabb_min_x]
    bufferOffset = _serializer.float64(obj.aabb_min_x, buffer, bufferOffset);
    // Serialize message field [aabb_max_x]
    bufferOffset = _serializer.float64(obj.aabb_max_x, buffer, bufferOffset);
    // Serialize message field [aabb_min_y]
    bufferOffset = _serializer.float64(obj.aabb_min_y, buffer, bufferOffset);
    // Serialize message field [aabb_max_y]
    bufferOffset = _serializer.float64(obj.aabb_max_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResolutionStatus
    let len;
    let data = new ResolutionStatus(null);
    // Deserialize message field [ldr_id]
    data.ldr_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [uav_ids]
    data.uav_ids = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [in_resolution]
    data.in_resolution = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [aabb_min_x]
    data.aabb_min_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [aabb_max_x]
    data.aabb_max_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [aabb_min_y]
    data.aabb_min_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [aabb_max_y]
    data.aabb_max_y = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.uav_ids.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 41;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uav_control/ResolutionStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68d9e3003f30b6e4832b7d80035318cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ResolutionStatus.msg
    # LDR Resolution状态消息定义
    
    
    # LDR标识符
    int32 ldr_id
    
    # 涉及的无人机ID列表
    string[] uav_ids
    
    # Resolution状态：true表示进入Resolution阶段，false表示退出
    bool in_resolution
    
    
    # LDR区域边界信息
    float64 aabb_min_x
    float64 aabb_max_x  
    float64 aabb_min_y
    float64 aabb_max_y
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResolutionStatus(null);
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

    if (msg.in_resolution !== undefined) {
      resolved.in_resolution = msg.in_resolution;
    }
    else {
      resolved.in_resolution = false
    }

    if (msg.aabb_min_x !== undefined) {
      resolved.aabb_min_x = msg.aabb_min_x;
    }
    else {
      resolved.aabb_min_x = 0.0
    }

    if (msg.aabb_max_x !== undefined) {
      resolved.aabb_max_x = msg.aabb_max_x;
    }
    else {
      resolved.aabb_max_x = 0.0
    }

    if (msg.aabb_min_y !== undefined) {
      resolved.aabb_min_y = msg.aabb_min_y;
    }
    else {
      resolved.aabb_min_y = 0.0
    }

    if (msg.aabb_max_y !== undefined) {
      resolved.aabb_max_y = msg.aabb_max_y;
    }
    else {
      resolved.aabb_max_y = 0.0
    }

    return resolved;
    }
};

module.exports = ResolutionStatus;
