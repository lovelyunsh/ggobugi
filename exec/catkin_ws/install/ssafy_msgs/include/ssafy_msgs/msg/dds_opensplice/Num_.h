//******************************************************************
// 
//  Generated by IDL to C++ Translator
//  
//  File name: Num_.h
//  Source: ssafy_msgs\msg\Num_.idl
//  Generated: timestamp removed to make the build reproducible
//  OpenSplice 6.9.190403OSS
//  
//******************************************************************
#ifndef _NUM__H_
#define _NUM__H_

#include "sacpp_mapping.h"
#include "ssafy_msgs/msg/rosidl_typesupport_opensplice_cpp__visibility_control.h"


namespace ssafy_msgs
{
   namespace msg
   {
      namespace dds_
      {
         struct Num_;

         struct ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs Num_
         {
               DDS::LongLong num_;
               DDS::Boolean air_;
               DDS::Boolean door_;
         };

         typedef DDS_DCPSStruct_var < Num_> Num__var;
         typedef Num_&Num__out;
      }
   }
}




#endif
