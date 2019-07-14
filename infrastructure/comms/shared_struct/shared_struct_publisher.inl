
namespace jet {

namespace {
const int O_FLAGS = O_RDWR | O_CREAT;
const off_t LENGTH = 921650 + 1024;
}

SharedStructPublisher::SharedStructPublisher(const std::string& object_name) : Publisher(object_name) {
  shmem_file_descriptor_ = shm_open(object_name.c_str(), O_FLAGS, 0644 );
  if (shmem_file_descriptor_ < 0) {
    throw std::runtime_error("Could not open shared memory region.");
  }

  int result = ftruncate(shmem_file_descriptor_, LENGTH);
  if (result < 0) {
    throw std::runtime_error("ftruncate filed while opening shared memory region.");
  }
  shmem_region_ptr_ = (SharedStructMemoryRegion *) mmap(NULL, LENGTH, PROT_READ | PROT_WRITE, MAP_SHARED, shmem_file_descriptor_, 0);
  assert(shmem_region_ptr_);
}

SharedStructPublisher::~SharedStructPublisher() {
  close(shmem_file_descriptor_);
}

void SharedStructPublisher::publish(Message& message) {
  ++sequence_number_;
  message.header.sequence_number = sequence_number_;
  message.header.timestamp_ns = time::get_current_time();
  std::string data;
  message.serialize(data);
  publish_raw(data);
}

void SharedStructPublisher::publish_raw(const std::string& data) {
  char * message_buffer_pointer = shmem_region_ptr_->message;
  shmem_region_ptr_->region_futex.lock();
  memcpy(message_buffer_pointer, data.c_str(), data.size());
  shmem_region_ptr_->message_len = data.size();
  shmem_region_ptr_->region_futex.unlock();
}

}  // namespace jet
