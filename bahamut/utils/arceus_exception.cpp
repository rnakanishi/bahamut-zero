#include <utils/exception.hpp>
namespace Arceus {

ArceusException::ArceusException()
    : ArceusException("Exception occurred", "") {}

ArceusException::ArceusException(std::string msg, std::string location) {
  _errorMessage = msg;
  _callingLocation = location;
  _exceptionName = "Arceus_Exception";
  _errorNumber = 100;
}

ArceusException::ArceusException(int errorId, std::string location) {
  _callingLocation = location;
  _exceptionName = "Arceus_Exception";
}

int ArceusException::getErrorNumber() const throw() {
  return _errorNumber;
}

const char* ArceusException::getErrorType() const throw() {
  return _exceptionName.c_str();
}

const char* ArceusException::what() const throw() {
  std::string error;
  if (_isLethal)
    error = "\033[1;31m[ERROR]\033[0m ";
  else
    error = "\033[21;33m[WARNING]: \033[0m";

  error.append(_errorMessage);
  error.append("\n");
  error.append("Location: ");
  error.append(_callingLocation);
  error.append("\n");
  return error.c_str();
}

void ArceusException::appendMessage(std::string message) {
  _errorMessage.append("\n-----\n");
  _errorMessage.append(message);
}

void ArceusException::setLethality(bool lethality) {
  _isLethal = lethality;
}

const bool& ArceusException::isLethal() {
  return _isLethal;
}

void ArceusException::setErrorNumber(int errorNumber, std::string location) {
  _errorNumber = errorNumber;
  switch (errorNumber) {
    case 100:
      _errorMessage = "Exception occured";
      break;
    case 101:
      _errorMessage = "Invalid Exception Id";
      break;
    case 102:
      _errorMessage = "Method not implemented";
      break;
    default:
      throw(ArceusException(101, location));
      break;
  }
}

void ArceusException::setCallingLocation(std::string location) {
  _callingLocation = location;
}

}  // namespace Arceus
