#ifndef BBOX_DCPS_IMPL_H_
#define BBOX_DCPS_IMPL_H_

#include "ccpp.h"
#include "ccpp_BBox_.h"
#include "TypeSupportMetaHolder.h"
#include "TypeSupport.h"
#include "FooDataWriter_impl.h"
#include "FooDataReader_impl.h"
#include "FooDataReaderView_impl.h"
#include "ssafy_msgs/msg/rosidl_typesupport_opensplice_cpp__visibility_control.h"

namespace ssafy_msgs {

    namespace msg {

        namespace dds_ {

            class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs BBox_TypeSupportMetaHolder : public ::DDS::OpenSplice::TypeSupportMetaHolder
            {
            public:
                BBox_TypeSupportMetaHolder ();
                virtual ~BBox_TypeSupportMetaHolder ();
            
            private:
                ::DDS::OpenSplice::TypeSupportMetaHolder * clone();
            
                ::DDS::OpenSplice::DataWriter * create_datawriter ();
            
                ::DDS::OpenSplice::DataReader * create_datareader ();
            
                ::DDS::OpenSplice::DataReaderView * create_view ();
            };
            
            class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs BBox_TypeSupport : public virtual BBox_TypeSupportInterface,
                                                               public ::DDS::OpenSplice::TypeSupport
            {
            public:
                BBox_TypeSupport ();
            
                virtual ~BBox_TypeSupport ();
            
            private:
                BBox_TypeSupport (const BBox_TypeSupport &);
            
                BBox_TypeSupport & operator= (const BBox_TypeSupport &);
            };
            
            typedef BBox_TypeSupportInterface_var BBox_TypeSupport_var;
            typedef BBox_TypeSupportInterface_ptr BBox_TypeSupport_ptr;
            
            class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs BBox_DataWriter_impl : public virtual BBox_DataWriter,
                                                                   public ::DDS::OpenSplice::FooDataWriter_impl
            {
                friend class DDS::OpenSplice::Publisher;
                friend class BBox_TypeSupportMetaHolder;
            
            public:
                virtual ::DDS::InstanceHandle_t register_instance (
                    const BBox_ & instance_data) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::InstanceHandle_t register_instance_w_timestamp (
                    const BBox_ & instance_data,
                    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t unregister_instance (
                    const BBox_ & instance_data,
                    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t unregister_instance_w_timestamp (
                    const BBox_ & instance_data,
                    ::DDS::InstanceHandle_t handle,
                    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t write (
                    const BBox_ & instance_data,
                    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t write_w_timestamp (
                    const BBox_ & instance_data,
                    ::DDS::InstanceHandle_t handle,
                    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t dispose (
                    const BBox_ & instance_data,
                    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t dispose_w_timestamp (
                    const BBox_ & instance_data,
                    ::DDS::InstanceHandle_t handle,
                    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t writedispose (
                    const BBox_ & instance_data,
                    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t writedispose_w_timestamp (
                    const BBox_ & instance_data,
                    ::DDS::InstanceHandle_t handle,
                    const ::DDS::Time_t & source_timestamp) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t get_key_value (
                    BBox_ & key_holder,
                    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::InstanceHandle_t lookup_instance (
                    const BBox_ & instance_data) THROW_ORB_EXCEPTIONS;
            
            protected:
                BBox_DataWriter_impl ();
            
                virtual ~BBox_DataWriter_impl ();
            
                virtual DDS::ReturnCode_t init (
                                DDS::OpenSplice::Publisher *publisher,
                                DDS::OpenSplice::DomainParticipant *participant,
                                const DDS::DataWriterQos &qos,
                                DDS::OpenSplice::Topic *a_topic,
                                const char *name,
                                DDS::OpenSplice::cxxCopyIn copyIn,
                                DDS::OpenSplice::cxxCopyOut copyOut,
                                u_writerCopy writerCopy,
                                void *cdrMarshaler);
            
            private:
                BBox_DataWriter_impl (const BBox_DataWriter_impl &);
            
                BBox_DataWriter_impl & operator= (const BBox_DataWriter_impl &);
            };
            
            class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs BBox_DataReader_impl : public virtual BBox_DataReader,
                                                                   public ::DDS::OpenSplice::FooDataReader_impl
            {
                friend class DDS::OpenSplice::Subscriber;
                friend class BBox_TypeSupportMetaHolder;
                friend class BBox_DataReaderView_impl;
            
            public:
                virtual ::DDS::ReturnCode_t read (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t read_w_condition (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take_w_condition (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t read_next_sample (
                    BBox_ & received_data,
                    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take_next_sample (
                    BBox_ & received_data,
                    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t read_instance (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take_instance (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t read_next_instance (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take_next_instance (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t read_next_instance_w_condition (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take_next_instance_w_condition (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t return_loan (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t get_key_value (
                    BBox_ & key_holder,
                    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::InstanceHandle_t lookup_instance (
                    const BBox_ & instance) THROW_ORB_EXCEPTIONS;
            
            protected:
                BBox_DataReader_impl ();
            
                virtual ~BBox_DataReader_impl ();
            
                DDS::ReturnCode_t init (
                        DDS::OpenSplice::Subscriber *subscriber,
                        const DDS::DataReaderQos &qos,
                        DDS::OpenSplice::TopicDescription *a_topic,
                        const char *name,
                        DDS::OpenSplice::cxxCopyIn copyIn,
                        DDS::OpenSplice::cxxCopyOut copyOut,
                        DDS::OpenSplice::cxxReaderCopy readerCopy,
                        void *cdrMarshaler);
            
                static void* dataSeqAlloc (
                    void * data_values,
                    DDS::ULong len);
            
                static void dataSeqLength (
                    void * data_values,
                    DDS::ULong len);
            
                static void * dataSeqGetBuffer (
                    void * data_values,
                    DDS::ULong index);
            
                static void dataSeqCopyOut (
                    const void * from,
                    void * received_data);
                static void copyDataOut(const void *from, void *to);
            
            private:
                BBox_DataReader_impl (const BBox_DataReader_impl &);
                BBox_DataReader_impl & operator= (const BBox_DataReader_impl &);
            
                static ::DDS::ReturnCode_t check_preconditions (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples);
            };
            
            class ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_ssafy_msgs BBox_DataReaderView_impl : public virtual BBox_DataReaderView,
                                                                       public ::DDS::OpenSplice::FooDataReaderView_impl
            {
                friend class DDS::OpenSplice::DataReader;
                friend class BBox_TypeSupportMetaHolder;
            
            public:
                virtual ::DDS::ReturnCode_t read (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t read_w_condition (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take_w_condition (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t read_next_sample (
                    BBox_ & received_data,
                    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take_next_sample (
                    BBox_ & received_data,
                    ::DDS::SampleInfo & sample_info) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t read_instance (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take_instance (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t read_next_instance (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take_next_instance (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::SampleStateMask sample_states,
                    ::DDS::ViewStateMask view_states,
                    ::DDS::InstanceStateMask instance_states) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t read_next_instance_w_condition (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t take_next_instance_w_condition (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq,
                    ::DDS::Long max_samples,
                    ::DDS::InstanceHandle_t a_handle,
                    ::DDS::ReadCondition_ptr a_condition) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t return_loan (
                    BBox_Seq & received_data,
                    ::DDS::SampleInfoSeq & info_seq) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::ReturnCode_t get_key_value (
                    BBox_ & key_holder,
                    ::DDS::InstanceHandle_t handle) THROW_ORB_EXCEPTIONS;
            
                virtual ::DDS::InstanceHandle_t lookup_instance (
                    const BBox_ & instance) THROW_ORB_EXCEPTIONS;
            
            protected:
                BBox_DataReaderView_impl ();
            
                virtual ~BBox_DataReaderView_impl ();
            
                virtual DDS::ReturnCode_t init (
                    DDS::OpenSplice::DataReader *reader,
                    const char *name,
                    const DDS::DataReaderViewQos &qos,
                    DDS::OpenSplice::cxxCopyIn copyIn,
                    DDS::OpenSplice::cxxCopyOut copyOut);
            
            private:
                BBox_DataReaderView_impl (const BBox_DataReaderView_impl &);
            
                BBox_DataReaderView_impl & operator= (const BBox_DataReaderView_impl &);
            };
            
        }

    }

}

#undef OS_API
#endif
