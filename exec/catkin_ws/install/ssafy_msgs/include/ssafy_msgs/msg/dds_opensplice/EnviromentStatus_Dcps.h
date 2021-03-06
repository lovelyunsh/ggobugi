//******************************************************************
// 
//  Generated by IDL to C++ Translator
//  
//  File name: EnviromentStatus_Dcps.h
//  Source: ssafy_msgs\msg\EnviromentStatus_.idl
//  Generated: timestamp removed to make the build reproducible
//  OpenSplice 6.9.190403OSS
//  
//******************************************************************
#ifndef _ENVIROMENTSTATUS_DCPS_H_
#define _ENVIROMENTSTATUS_DCPS_H_

#include "sacpp_mapping.h"
#include "dds_dcps.h"
#include "EnviromentStatus_.h"
#include "ssafy_msgs/msg/rosidl_typesupport_opensplice_cpp__visibility_control.h"


namespace ssafy_msgs
{
   namespace msg
   {
      namespace dds_
      {

         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs EnviromentStatus_TypeSupportInterface;

         typedef EnviromentStatus_TypeSupportInterface * EnviromentStatus_TypeSupportInterface_ptr;
         typedef DDS_DCPSInterface_var < EnviromentStatus_TypeSupportInterface> EnviromentStatus_TypeSupportInterface_var;
         typedef DDS_DCPSInterface_out < EnviromentStatus_TypeSupportInterface> EnviromentStatus_TypeSupportInterface_out;


         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs EnviromentStatus_DataWriter;

         typedef EnviromentStatus_DataWriter * EnviromentStatus_DataWriter_ptr;
         typedef DDS_DCPSInterface_var < EnviromentStatus_DataWriter> EnviromentStatus_DataWriter_var;
         typedef DDS_DCPSInterface_out < EnviromentStatus_DataWriter> EnviromentStatus_DataWriter_out;


         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs EnviromentStatus_DataReader;

         typedef EnviromentStatus_DataReader * EnviromentStatus_DataReader_ptr;
         typedef DDS_DCPSInterface_var < EnviromentStatus_DataReader> EnviromentStatus_DataReader_var;
         typedef DDS_DCPSInterface_out < EnviromentStatus_DataReader> EnviromentStatus_DataReader_out;


         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs EnviromentStatus_DataReaderView;

         typedef EnviromentStatus_DataReaderView * EnviromentStatus_DataReaderView_ptr;
         typedef DDS_DCPSInterface_var < EnviromentStatus_DataReaderView> EnviromentStatus_DataReaderView_var;
         typedef DDS_DCPSInterface_out < EnviromentStatus_DataReaderView> EnviromentStatus_DataReaderView_out;

         struct EnviromentStatus_Seq_uniq_ {};
         typedef DDS_DCPSUVLSeq < EnviromentStatus_, struct EnviromentStatus_Seq_uniq_> EnviromentStatus_Seq;
         typedef DDS_DCPSSequence_var < EnviromentStatus_Seq> EnviromentStatus_Seq_var;
         typedef DDS_DCPSSequence_out < EnviromentStatus_Seq> EnviromentStatus_Seq_out;
         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs EnviromentStatus_TypeSupportInterface
         :
            virtual public DDS::TypeSupport
         { 
         public:
            typedef EnviromentStatus_TypeSupportInterface_ptr _ptr_type;
            typedef EnviromentStatus_TypeSupportInterface_var _var_type;

            static EnviromentStatus_TypeSupportInterface_ptr _duplicate (EnviromentStatus_TypeSupportInterface_ptr obj);
            DDS::Boolean _local_is_a (const char * id);

            static EnviromentStatus_TypeSupportInterface_ptr _narrow (DDS::Object_ptr obj);
            static EnviromentStatus_TypeSupportInterface_ptr _unchecked_narrow (DDS::Object_ptr obj);
            static EnviromentStatus_TypeSupportInterface_ptr _nil () { return 0; }
            static const char * _local_id;
            EnviromentStatus_TypeSupportInterface_ptr _this () { return this; }


         protected:
            EnviromentStatus_TypeSupportInterface () {};
            ~EnviromentStatus_TypeSupportInterface () {};
         private:
            EnviromentStatus_TypeSupportInterface (const EnviromentStatus_TypeSupportInterface &);
            EnviromentStatus_TypeSupportInterface & operator = (const EnviromentStatus_TypeSupportInterface &);
         };

         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs EnviromentStatus_DataWriter
         :
            virtual public DDS::DataWriter
         { 
         public:
            typedef EnviromentStatus_DataWriter_ptr _ptr_type;
            typedef EnviromentStatus_DataWriter_var _var_type;

            static EnviromentStatus_DataWriter_ptr _duplicate (EnviromentStatus_DataWriter_ptr obj);
            DDS::Boolean _local_is_a (const char * id);

            static EnviromentStatus_DataWriter_ptr _narrow (DDS::Object_ptr obj);
            static EnviromentStatus_DataWriter_ptr _unchecked_narrow (DDS::Object_ptr obj);
            static EnviromentStatus_DataWriter_ptr _nil () { return 0; }
            static const char * _local_id;
            EnviromentStatus_DataWriter_ptr _this () { return this; }

            virtual DDS::LongLong register_instance (const EnviromentStatus_& instance_data) = 0;
            virtual DDS::LongLong register_instance_w_timestamp (const EnviromentStatus_& instance_data, const DDS::Time_t& source_timestamp) = 0;
            virtual DDS::Long unregister_instance (const EnviromentStatus_& instance_data, DDS::LongLong handle) = 0;
            virtual DDS::Long unregister_instance_w_timestamp (const EnviromentStatus_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
            virtual DDS::Long write (const EnviromentStatus_& instance_data, DDS::LongLong handle) = 0;
            virtual DDS::Long write_w_timestamp (const EnviromentStatus_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
            virtual DDS::Long dispose (const EnviromentStatus_& instance_data, DDS::LongLong handle) = 0;
            virtual DDS::Long dispose_w_timestamp (const EnviromentStatus_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
            virtual DDS::Long writedispose (const EnviromentStatus_& instance_data, DDS::LongLong handle) = 0;
            virtual DDS::Long writedispose_w_timestamp (const EnviromentStatus_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
            virtual DDS::Long get_key_value (EnviromentStatus_& key_holder, DDS::LongLong handle) = 0;
            virtual DDS::LongLong lookup_instance (const EnviromentStatus_& instance_data) = 0;

         protected:
            EnviromentStatus_DataWriter () {};
            ~EnviromentStatus_DataWriter () {};
         private:
            EnviromentStatus_DataWriter (const EnviromentStatus_DataWriter &);
            EnviromentStatus_DataWriter & operator = (const EnviromentStatus_DataWriter &);
         };

         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs EnviromentStatus_DataReader
         :
            virtual public DDS::DataReader
         { 
         public:
            typedef EnviromentStatus_DataReader_ptr _ptr_type;
            typedef EnviromentStatus_DataReader_var _var_type;

            static EnviromentStatus_DataReader_ptr _duplicate (EnviromentStatus_DataReader_ptr obj);
            DDS::Boolean _local_is_a (const char * id);

            static EnviromentStatus_DataReader_ptr _narrow (DDS::Object_ptr obj);
            static EnviromentStatus_DataReader_ptr _unchecked_narrow (DDS::Object_ptr obj);
            static EnviromentStatus_DataReader_ptr _nil () { return 0; }
            static const char * _local_id;
            EnviromentStatus_DataReader_ptr _this () { return this; }

            virtual DDS::Long read (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_w_condition (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long take_w_condition (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long read_next_sample (EnviromentStatus_& received_data, DDS::SampleInfo& sample_info) = 0;
            virtual DDS::Long take_next_sample (EnviromentStatus_& received_data, DDS::SampleInfo& sample_info) = 0;
            virtual DDS::Long read_instance (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take_instance (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_next_instance (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take_next_instance (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_next_instance_w_condition (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long take_next_instance_w_condition (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long return_loan (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq) = 0;
            virtual DDS::Long get_key_value (EnviromentStatus_& key_holder, DDS::LongLong handle) = 0;
            virtual DDS::LongLong lookup_instance (const EnviromentStatus_& instance) = 0;

         protected:
            EnviromentStatus_DataReader () {};
            ~EnviromentStatus_DataReader () {};
         private:
            EnviromentStatus_DataReader (const EnviromentStatus_DataReader &);
            EnviromentStatus_DataReader & operator = (const EnviromentStatus_DataReader &);
         };

         class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs EnviromentStatus_DataReaderView
         :
            virtual public DDS::DataReaderView
         { 
         public:
            typedef EnviromentStatus_DataReaderView_ptr _ptr_type;
            typedef EnviromentStatus_DataReaderView_var _var_type;

            static EnviromentStatus_DataReaderView_ptr _duplicate (EnviromentStatus_DataReaderView_ptr obj);
            DDS::Boolean _local_is_a (const char * id);

            static EnviromentStatus_DataReaderView_ptr _narrow (DDS::Object_ptr obj);
            static EnviromentStatus_DataReaderView_ptr _unchecked_narrow (DDS::Object_ptr obj);
            static EnviromentStatus_DataReaderView_ptr _nil () { return 0; }
            static const char * _local_id;
            EnviromentStatus_DataReaderView_ptr _this () { return this; }

            virtual DDS::Long read (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_w_condition (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long take_w_condition (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long read_next_sample (EnviromentStatus_& received_data, DDS::SampleInfo& sample_info) = 0;
            virtual DDS::Long take_next_sample (EnviromentStatus_& received_data, DDS::SampleInfo& sample_info) = 0;
            virtual DDS::Long read_instance (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take_instance (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_next_instance (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long take_next_instance (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
            virtual DDS::Long read_next_instance_w_condition (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long take_next_instance_w_condition (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
            virtual DDS::Long return_loan (EnviromentStatus_Seq& received_data, DDS::SampleInfoSeq& info_seq) = 0;
            virtual DDS::Long get_key_value (EnviromentStatus_& key_holder, DDS::LongLong handle) = 0;
            virtual DDS::LongLong lookup_instance (const EnviromentStatus_& instance) = 0;

         protected:
            EnviromentStatus_DataReaderView () {};
            ~EnviromentStatus_DataReaderView () {};
         private:
            EnviromentStatus_DataReaderView (const EnviromentStatus_DataReaderView &);
            EnviromentStatus_DataReaderView & operator = (const EnviromentStatus_DataReaderView &);
         };

      }
   }
}




#endif
