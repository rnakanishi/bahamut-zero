#include <exceptions/exception.hpp>

namespace Bahamut {
UnexpectedParameterException::UnexpectedParameterException()
    : UnexpectedParameterException("Unexpected parameter exception", "") {}

UnexpectedParameterException::UnexpectedParameterException(
    std::string msg,
    std::string location) {
  _errorMessage = msg;
  _callingLocation = location;
  _exceptionName = "Bahamut_Exception::Unexpected_Parameter";
  _errorNumber = 300;
}

UnexpectedParameterException::UnexpectedParameterException(
    int errorId,
    std::string location) {
  _callingLocation = location;
  _exceptionName = "Bahamut_Exception::Unexpected_Parameter";
  setErrorNumber(errorId, location);
}

void UnexpectedParameterException::setErrorNumber(int errorNumber,
                                                  std::string location) {
  _errorNumber = errorNumber;
  switch (errorNumber) {
    case 300:
      _errorMessage = "Unexpected parameter exception";
      break;
    case 301:
      _errorMessage = "Polynomial degree not supported";
      break;
    case 302:
      _errorMessage = "Vector sizes does not match";
      break;
    case 303:
      _errorMessage = "Derivative degree not supported";
      break;
    case 304:
      _errorMessage = "Enum parameter not supported";
      break;
    case 305:
      _errorMessage = "Enum parameter not implemented";
      break;
    case 306:
      _errorMessage = "Invalid cell id";
      break;
    case 307:
      _errorMessage = "Invalid particle id";
      break;
    case 308:
      _errorMessage = "Not active particle";
      break;
    case 309:
      _errorMessage = "Invalid edge Id";
      break;
    default:
      std::string myLocation("UnexpectedParameterException::constructor\n");
      myLocation.append(location);
      throw(BahamutException(101, myLocation));
      break;
  }
}

}  // namespace Bahamut
