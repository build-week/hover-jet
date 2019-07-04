
namespace jet {

SharedStructPublisher::SharedStructPublisher(const std::string& object_name) : Publisher(object_name) {
  int oflags = O_RDWR | O_CREAT;
  off_t length = 921650 + 1024;
  int fd_ = shm_open(object_name.c_str(), oflags, 0644 );
  if (fd_ < 0) {
    std::cerr << "Error " <<  errno << ": " << errno << std::endl;
  }

  ftruncate(fd_, length);
  shmem_region_ptr_ = (SharedStructMemoryRegion *) mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  assert(shmem_region_ptr_);
}

SharedStructPublisher::~SharedStructPublisher() {
  close(fd_);
}

void SharedStructPublisher::publish(Message& message) {
  ++sequence_number_;
  message.header.sequence_number = sequence_number_;
  message.header.timestamp_ns = time::get_current_time();
  std::string data;
  message.serialize(data);

  // std::cout << "SIZE: " << data.size() << std::endl;

  publish_raw(data);
}

void SharedStructPublisher::publish_raw(const std::string& data) {
  // Mutex * region_futex_ptr = &(shmem_region_ptr_->region_futex);
  char * message_buffer_pointer = shmem_region_ptr_->message;
  // std::cerr << shmem_region_ptr_ << std::endl;
  // std::cerr << &(shmem_region_ptr_->message) << std::endl;
  // std::cerr << region_futex_ptr << std::endl;


  shmem_region_ptr_->region_futex.lock();
  // strcpy((char*)message_buffer_pointer, data.c_str());
  memcpy(message_buffer_pointer, data.c_str(), data.size());
  shmem_region_ptr_->message_len = data.size();
  shmem_region_ptr_->region_futex.unlock();
}

}  // namespace jet
