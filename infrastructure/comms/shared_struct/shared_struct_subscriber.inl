
namespace jet {

SharedStructSubscriber::SharedStructSubscriber(const std::string& object_name) : Subscriber(object_name) {
  int oflags = O_RDWR;
  int fd_ = shm_open(object_name.c_str(), oflags, 0644);
  if (fd_ < 0) {
    std::cerr << "Error " <<  errno << ": " << errno << std::endl;
  }

  struct stat sb;
  fstat(fd_, &sb);
  off_t length = sb.st_size ;
  shmem_region_ptr_ = (SharedStructMemoryRegion *) mmap(NULL, length, PROT_READ|PROT_WRITE, MAP_SHARED, fd_, 0);
  assert(shmem_region_ptr_);
}

SharedStructSubscriber::~SharedStructSubscriber() {
  close(fd_);
}

bool SharedStructSubscriber::read(Message& message, const Duration& timeout) {
  std::string data;
  if (!read_raw(data, timeout)) {
    return false;
  }
  message.deserialize(data);
  if (message.header.timestamp_ns == 0) {
    return false;
  }
  if (message.header.sequence_number == most_recent_sequence_number_ && message.header.timestamp_ns == most_recent_timestamp_) {
    return false;
  }
  most_recent_sequence_number_ = message.header.sequence_number;
  most_recent_timestamp_ = message.header.timestamp_ns;
  return true;
}

bool SharedStructSubscriber::read_raw(std::string& data, const Duration& timeout) {
  (void) timeout;
  if (!shmem_region_ptr_->region_futex.lock(0)) {
    return false;
  }
  data.assign(shmem_region_ptr_->message, shmem_region_ptr_->message_len);
  shmem_region_ptr_->region_futex.unlock();
  return true;
}

bool SharedStructSubscriber::read_latest(Message& message, const Duration& timeout) {
  (void) timeout;
  return read(message, 0);
}

}  // namespace jet
