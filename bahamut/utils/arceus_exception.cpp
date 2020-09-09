#include <utils/exception.hpp>
namespace Bahamut {

BahamutException::BahamutException()
    : BahamutException("Exception occurred", "") {}

BahamutException::BahamutException(std::string msg, std::string location) {
  _errorMessage = msg;
  _callingLocation = location;
  _exceptionName = "Bahamut_Exception";
  _errorNumber = 100;
}

BahamutException::BahamutException(int errorId, std::string location) {
  _callingLocation = location;
  _exceptionName = "Bahamut_Exception";
}

int BahamutException::getErrorNumber() const throw() {
  return _errorNumber;
}

const char* BahamutException::getErrorType() const throw() {
  return _exceptionName.c_str();
}

const char* BahamutException::what() const throw() {
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

void BahamutException::appendMessage(std::string message) {
  _errorMessage.append("\n-----\n");
  _errorMessage.append(message);
}

void BahamutException::setLethality(bool lethality) {
  _isLethal = lethality;
}

const bool& BahamutException::isLethal() {
  return _isLethal;
}

void BahamutException::setErrorNumber(int errorNumber, std::string location) {
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
      throw(BahamutException(101, location));
      break;
  }
}

void BahamutException::setCallingLocation(std::string location) {
  _callingLocation = location;
}

}  // namespace Bahamut
