#include <exceptions/exception.hpp>

namespace Bahamut {
ConditionsNotMatchException::ConditionsNotMatchException()
    : ConditionsNotMatchException("Conditions not match", "") {}

ConditionsNotMatchException::ConditionsNotMatchException(std::string msg,
                                                         std::string location) {
  _errorMessage = msg;
  _callingLocation = location;
  _exceptionName = "Conditions_Not_Match";
  _errorNumber = 400;
  _isLethal = true;
}

ConditionsNotMatchException::ConditionsNotMatchException(int errorId,
                                                         std::string location) {
  _callingLocation = location;
  _exceptionName = "Conditions_Not_Match";
  _isLethal = true;
  setErrorNumber(errorId, location);
}

void ConditionsNotMatchException::setErrorNumber(int errorNumber,
                                                 std::string location) {
  _errorNumber = errorNumber;

  switch (errorNumber) {
    case 400:
      _errorMessage = "Conditions not match exception";
      break;
    case 401:
      _errorMessage = "Different type value already set";
      break;
    case 402:
      _errorMessage = "Too few sample points provided";
      break;
    case 403:
      _errorMessage = "Cell is not a surface";
      break;
    case 404:
      _errorMessage = "Horizontal direction does not have a surface";
      break;
    case 405:
      _errorMessage = "Vertical direction does not have a surface";
      break;
    case 406:
      _errorMessage = "Vertices list is not empty";
      break;
    default:
      std::string myLocation("ConditionsNotMatchExcetption::constructor\n");
      myLocation.append(location);
      throw(BahamutException(101, myLocation));
      break;
  }
}

}  // namespace Bahamut
