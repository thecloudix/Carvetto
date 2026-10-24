#include "carvetto/document/sheet.hpp"

namespace carvetto::document {

Sheet Sheet::fromJob(const Job& job, std::string sheetName) {
    Sheet sheet;
    sheet.name = std::move(sheetName);
    sheet.sizeMm = job.sizeMm;
    sheet.zZero = job.zZero;
    sheet.datum = job.datum;
    sheet.datumOffsetMm = job.datumOffsetMm;
    return sheet;
}

}  // namespace carvetto::document
