#include <utils/exception.hpp>

namespace Bahamut {
BadResultException::BadResultException()
    : BadResultException("Conditions not match", "") {}

BadResultException::BadResultException(std::string msg, std::string location) {
  _errorMessage = msg;
  _callingLocation = location;
  _exceptionName = "Bad_Result_Exception";
  _errorNumber = 500;
  _isLethal = true;
}

BadResultException::BadResultException(int errorId, std::string location) {
  _callingLocation = location;
  _exceptionName = "Bad_Result_Exception";
  _isLethal = true;
  setErrorNumber(errorId, location);
}

void BadResultException::setErrorNumber(int errorNumber, std::string location) {
  _errorNumber = errorNumber;

  switch (errorNumber) {
    case 500:
      _errorMessage = "Conditions not match exception";
      break;
    case 501:
      _errorMessage = "Resulting vector contains NaN values";
      break;
    case 502:
      _errorMessage = "NaN scalar result";
      break;
    default:
      std::string myLocation("BadResult::constructor\n");
      myLocation.append(location);
      throw(BahamutException(101, myLocation));
      break;
  }
}

}  // namespace Bahamut
