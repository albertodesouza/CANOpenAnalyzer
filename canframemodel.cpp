#include "canframemodel.h"

#include <QFile>
#include <QApplication>
#include <QPalette>
#include <QDateTime>
#include "utility.h"

CANFrameModel::~CANFrameModel()
{
    frames.clear();
    filteredFrames.clear();
    filters.clear();
}

int CANFrameModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    if (filteredFrames.data())
    {
        int rows = filteredFrames.count();
        return rows;
    }

     //just in case somehow data is invalid which I have seen before.
    //But, this should not happen so issue a debugging message too
    qDebug() << "Invalid data for filteredFrames. Returning 0.";
    return 0;
}

int CANFrameModel::totalFrameCount()
{
    int count;
    count = frames.count();
    return count;
}

int CANFrameModel::columnCount(const QModelIndex &index) const
{
    Q_UNUSED(index);
    return (int)Column::NUM_COLUMN;
}

CANFrameModel::CANFrameModel(QObject *parent)
    : QAbstractTableModel(parent)
{

    if (QSysInfo::WordSize > 32)
    {
        qDebug() << "64 bit OS detected. Requesting a large preallocation";
        preallocSize = 10000000;
    }
    else //if compiling for 32 bit you can't ask for gigabytes of preallocation so tone it down.
    {
        qDebug() << "32 bit OS detected. Requesting a much restricted prealloc";
        preallocSize = 2000000;
    }

    frames.reserve(preallocSize);
    filteredFrames.reserve(preallocSize); //the goal is to prevent a reallocation from ever happening

    dbcHandler = DBCHandler::getReference();
    interpretFrames = false;
    overwriteDups = false;
    useHexMode = true;
    timeSeconds = false;
    timeOffset = 0;
    needFilterRefresh = false;
    lastUpdateNumFrames = 0;
    timeFormat =  "MMM-dd HH:mm:ss.zzz";
    sortDirAsc = false;
}

void CANFrameModel::setHexMode(bool mode)
{
    if (useHexMode != mode)
    {
        this->beginResetModel();
        useHexMode = mode;
        Utility::decimalMode = !useHexMode;
        this->endResetModel();
    }
}

void CANFrameModel::setSecondsMode(bool mode)
{
    if (Utility::secondsMode != mode)
    {
        this->beginResetModel();
        Utility::secondsMode = mode;
        this->endResetModel();
    }
}

void CANFrameModel::setSysTimeMode(bool mode)
{
    if (Utility::sysTimeMode != mode)
    {
        this->beginResetModel();
        Utility::sysTimeMode = mode;
        this->endResetModel();
    }
}

void CANFrameModel::setInterpretMode(bool mode)
{
    //if the state of interpretFrames changes then we need to reset the model
    //so that QT will refresh the view properly
    if (interpretFrames != mode)
    {
        this->beginResetModel();
        interpretFrames = mode;
        this->endResetModel();
    }
}

bool CANFrameModel::getInterpretMode()
{
    return interpretFrames;
}

void CANFrameModel::setTimeFormat(QString format)
{
    Utility::timeFormat = format;
    beginResetModel(); //reset model to show new time format
    endResetModel();
}

/*
 * Scan all frames for the smallest timestamp and offset all timestamps so that smallest one is at 0
*/
void CANFrameModel::normalizeTiming()
{
    mutex.lock();
    if (frames.count() == 0) return;
    timeOffset = frames[0].timeStamp().microSeconds();

    //find the absolute lowest timestamp in the whole time. Needed because maybe timestamp was reset in the middle.
    for (int j = 0; j < frames.count(); j++)
    {
        if (frames[j].timeStamp().microSeconds() < timeOffset) timeOffset = frames[j].timeStamp().microSeconds();
    }

    for (int i = 0; i < frames.count(); i++)
    {
        frames[i].setTimeStamp(QCanBusFrame::TimeStamp(0, frames[i].timeStamp().microSeconds() - timeOffset));
    }

    this->beginResetModel();
    for (int i = 0; i < filteredFrames.count(); i++)
    {
        filteredFrames[i].setTimeStamp(QCanBusFrame::TimeStamp(0, filteredFrames[i].timeStamp().microSeconds() - timeOffset));
    }
    this->endResetModel();

    mutex.unlock();
}

void CANFrameModel::setOverwriteMode(bool mode)
{
    beginResetModel();
    overwriteDups = mode;
    recalcOverwrite();
    endResetModel();
}

void CANFrameModel::setFilterState(unsigned int ID, bool state)
{
    if (!filters.contains(ID & 0x7F)) return;
    filters[ID] = state;
    sendRefresh();
}

void CANFrameModel::setAllFilters(bool state)
{
    QMap<int, bool>::iterator it;
    for (it = filters.begin(); it != filters.end(); ++it)
    {
        it.value() = state;
    }
    sendRefresh();
}

/*
 * There is probably a more correct way to have done this but below are several functions that collectively implement
 * quicksort on the columns and interpret the columns numerically. But, correct or not, this implementation is quite fast
 * and sorts the columns properly.
*/
uint64_t CANFrameModel::getCANFrameVal(int row, Column col)
{
    uint64_t temp = 0;
    if (row >= frames.count()) return 0;
    CANFrame frame = frames[row];
    switch (col)
    {
    case Column::TimeStamp:
        if (overwriteDups) return frame.timedelta;
        return frame.timeStamp().microSeconds();
    case Column::FrameId:
        return frame.frameId();
    case Column::Extended:
        if (frame.hasExtendedFrameFormat()) return 1;
        return 0;
    case Column::Remote:
        if (overwriteDups) return frame.frameCount;
        if (frame.frameType() == QCanBusFrame::RemoteRequestFrame) return 1;
        return 0;
    case Column::Direction:
        if (frame.isReceived) return 1;
        return 0;
    case Column::Bus:
        return static_cast<uint64_t>(frame.bus);
    case Column::Length:
        return static_cast<uint64_t>(frame.payload().length());
    case Column::ASCII: //sort both the same for now
    case Column::Data:
        for (int i = 0; i < frame.payload().length(); i++) temp += (static_cast<uint64_t>(frame.payload()[i]) << (56 - (8 * i)));
        //qDebug() << temp;
        return temp;

    case Column::CANOpenFunction:
        return frame.frameId() >> 7;
    case Column::CANOpenNode:
        return frame.frameId() & 0x7f;

    case Column::NUM_COLUMN:
        return 0;
    }
    return 0;
}

void CANFrameModel::qSortCANFrameAsc(QVector<CANFrame> *frames, Column column, int lowerBound, int upperBound)
{
    int p, i, j;
    qDebug() << "Lower " << lowerBound << " Upper" << upperBound;
    if (lowerBound < upperBound)
    {
        uint64_t piv = getCANFrameVal(lowerBound + (upperBound - lowerBound) / 2, column);
        i = lowerBound - 1;
        j = upperBound + 1;
        for (;;){
            do {
                i++;
            } while ((i < upperBound) && getCANFrameVal(i, column) < piv);

            do
            {
                j--;
            } while ((j > lowerBound) && getCANFrameVal(j, column) > piv);
            if (i < j) {
                CANFrame temp = frames->at(i);
                frames->replace(i, frames->at(j));
                frames->replace(j, temp);
            }
            else {p = j; break;}
        }

        qSortCANFrameAsc(frames, column, lowerBound, p);
        qSortCANFrameAsc(frames, column, p+1, upperBound);
    }
}

void CANFrameModel::qSortCANFrameDesc(QVector<CANFrame> *frames, Column column, int lowerBound, int upperBound)
{
    int p, i, j;
    qDebug() << "Lower " << lowerBound << " Upper" << upperBound;
    if (lowerBound < upperBound)
    {
        uint64_t piv = getCANFrameVal(lowerBound + (upperBound - lowerBound) / 2, column);
        i = lowerBound - 1;
        j = upperBound + 1;
        for (;;){
            do {
                i++;
            } while ((i < upperBound) && getCANFrameVal(i, column) > piv);

            do
            {
                j--;
            } while ((j > lowerBound) && getCANFrameVal(j, column) < piv);
            if (i < j) {
                CANFrame temp = frames->at(i);
                frames->replace(i, frames->at(j));
                frames->replace(j, temp);
            }
            else {p = j; break;}
        }

        qSortCANFrameDesc(frames, column, lowerBound, p);
        qSortCANFrameDesc(frames, column, p+1, upperBound);
    }
}

void CANFrameModel::sortByColumn(int column)
{
    sortDirAsc = !sortDirAsc;
    //beginResetModel();
    if (sortDirAsc) qSortCANFrameAsc(&frames, Column(column), 0, frames.count()-1);
    else qSortCANFrameDesc(&frames, Column(column), 0, frames.count()-1);
    //endResetModel();
    sendRefresh();
}

bool CANFrameModel::filterFrameConsideringFunction(int frame_id)
{
    // Returns false if it is to filter
    int func = (frame_id & 0x7FF) >> 7;

    switch (func)
    {
    case 0:
        if (this->filterNMTon)
            return (false);
        else
            return (true);
        break;
    case 1:
        if ((frame_id & 0x7F) == 0)
        {
            if (this->filterSYNCon)
                return (false);
            else
                return (true);
        }
        else
        {
            if (this->filterEMCYon)
                return (false);
            else
                return (true);
        }
        break;
    case 2:
        if (this->filterTIMEon)
            return (false);
        else
            return (true);
        break;
//    case 3:
//        return "T PDO1";
//        break;
//    case 4:
//        return "R PDO1";
//        break;
//    case 5:
//        return "T PDO2";
//        break;
//    case 6:
//        return "R PDO2";
//        break;
//    case 7:
//        return "T PDO3";
//        break;
//    case 8:
//        return "R PDO3";
//        break;
//    case 9:
//        return "T PDO4";
//        break;
//    case 10://1010
//        return "R PDO4";
//        break;
//    case 11://1011
//        return "T SDO";
//        break;
//    case 12://1100
//        return "R SDO";
//        break;
//    case 13://1101
//        return "???";
//        break;
    case 14://1110
        if (this->filterHBEATon)
            return (false);
        else
            return (true);
        break;
//    case 15://1111
//        return "LSS";
//        break;
    default:
        return (true);
    }
}

//End of custom sorting code

void CANFrameModel::recalcOverwrite()
{
    if (!overwriteDups) return; //no need to do a thing if mode is disabled

    qDebug() << "recalcOverwrite called in model";

    mutex.lock();
    beginResetModel();

    //Look at the current list of frames and turn it into just a list of unique IDs
    QHash<uint64_t, CANFrame> overWriteFrames;
    uint64_t idAugmented; //id in lower 29 bits, bus number shifted up 29 bits
    foreach(CANFrame frame, frames)
    {
        idAugmented = frame.frameId();
        idAugmented = idAugmented + (frame.bus << 29ull);
        if (!overWriteFrames.contains(idAugmented))
        {
            frame.timedelta = 0;
            frame.frameCount = 1;
            overWriteFrames.insert(idAugmented, frame);
        }
        else
        {
            frame.timedelta = frame.timeStamp().microSeconds() - overWriteFrames[idAugmented].timeStamp().microSeconds();
            frame.frameCount = overWriteFrames[idAugmented].frameCount + 1;
            overWriteFrames[idAugmented] = frame;
        }
    }
    //Then replace the old list of frames with just the unique list
    frames.clear();
    frames.append(overWriteFrames.values().toVector());

    filteredFrames.clear();
    filteredFrames.reserve(preallocSize);

    for (int i = 0; i < frames.count(); i++)
    {
        if (filters[frames[i].frameId() & 0x7F] && filterFrameConsideringFunction(frames[i].frameId()))
        {
            filteredFrames.append(frames[i]);
        }
    }

    endResetModel();
    mutex.unlock();
}

QString CANFrameModel::printIndexSubIndex(const unsigned char *data) const
{
    QString tempString;

    tempString.append("I [0x");
    tempString.append(QString::number(256 * data[2] + data[1], 16));
    tempString.append(" (");
    tempString.append(QString::number(256 * data[2] + data[1], 10));
    tempString.append(")] SI [");
    tempString.append(QString::number(data[3], 10));
    tempString.append("]");

    return (tempString);
}

QString CANFrameModel::printSDO(int sdo, const unsigned char *data) const
{
    QString tempString;

    if (sdo)
    {
        int ccs = data[0] >> 5;
        if (sdo == 1)   // T SDO -> Server
        {
            tempString.append("Server - ");
            tempString.append(printIndexSubIndex(data));

            switch (ccs)
            {
            case 0:
            case 1:
                if (ccs == 1)
                    tempString.append(", Download Domain Segment, ");
                else
                    tempString.append(", Upload Domain Segment, ");
                {
                    int t = (data[0] >> 4) & 0x1;
                    int n = (data[0] >> 1) & 0x7;
                    int c = data[0] & 0x1;
                    tempString.append("n = ");
                    tempString.append(QString::number(8-n, 10));
                    if (c)
                        tempString.append(", more to download");
                    if (t)
                        tempString.append(", t = 1");
                    else
                        tempString.append(", t = 0");
                }
                break;
            case 2:
            case 3:
                if (ccs == 3)
                    tempString.append(", Initiate Domain Download");
                else
                    tempString.append(", Initiate Domain Upload");
                {
                    int n = (data[0] >> 2) & 0x3;
                    int e = (data[0] >> 1) & 0x1;
                    int s = data[0] & 0x1;
                    if (e && s)
                    {
                        tempString.append(", expedited, n = ");
                        tempString.append(QString::number(4-n, 10));
                    }
                    else if (s)
                    {
                        tempString.append(QString::number(data[4] + 256 * data[7], 10));
                    }
                }
                break;
            case 4:
                tempString.append(", Abort Domain Transfer");
                break;
            case 6:
                tempString.append(", Initiate Block Download");
                break;
            }
        }
        else            // R SDO -> Client
        {
            tempString.append("Client - ");
            tempString.append(printIndexSubIndex(data));

            switch (ccs)
            {
            case 0:
            case 3:
                if (ccs == 0)
                    tempString.append(", Download Domain Segment, ");
                else
                    tempString.append(", Upload Domain Segment, ");
                {
                    int t = (data[0] >> 4) & 0x1;
                    int n = (data[0] >> 1) & 0x7;
                    int c = data[0] & 0x1;
                    tempString.append("n = ");
                    tempString.append(QString::number(8-n, 10));
                    if (c)
                        tempString.append(", more to download");
                    if (t)
                        tempString.append(", t = 1");
                    else
                        tempString.append(", t = 0");
                }
                break;
            case 1:
            case 2:
                if (ccs == 1)
                    tempString.append(", Initiate Domain Download");
                else
                    tempString.append(", Initiate Domain Upload");
                {
                    int n = (data[0] >> 2) & 0x3;
                    int e = (data[0] >> 1) & 0x1;
                    int s = data[0] & 0x1;
                    if (e && s)
                    {
                        tempString.append(", expedited, n = ");
                        tempString.append(QString::number(4-n, 10));
                    }
                    else if (s)
                    {
                        tempString.append(QString::number(data[4] + 256 * data[7], 10));
                    }
                }
                break;
            case 4:
                tempString.append(", Abort Domain Transfer");
                break;
            case 6:
                tempString.append(", Initiate Block Download");
                break;
            }
        }
    }

    return (tempString);
}

QVariant CANFrameModel::data(const QModelIndex &index, int role) const
{
    QString tempString;
    CANFrame thisFrame;
    static bool rowFlip = false;
    QVariant ts;

    if (!index.isValid())
        return QVariant();

    if (index.row() >= (filteredFrames.count()))
        return QVariant();

    thisFrame = filteredFrames.at(index.row());

    const unsigned char *data = reinterpret_cast<const unsigned char *>(thisFrame.payload().constData());
    int dataLen = thisFrame.payload().count();

    if (role == Qt::BackgroundColorRole)
    {
        if (dbcHandler != nullptr && interpretFrames)
        {
            DBC_MESSAGE *msg = dbcHandler->findMessage(thisFrame);
            if (msg != nullptr)
            {
                return msg->bgColor;
            }
        }
        rowFlip = (index.row() % 2);
        if (rowFlip) return QApplication::palette().color(QPalette::Base);
        else return QApplication::palette().color(QPalette::AlternateBase);
    }

    if (role == Qt::TextAlignmentRole)
    {
        switch(Column(index.column()))
        {
        case Column::TimeStamp:
            return Qt::AlignRight;
        case Column::FrameId:
        case Column::CANOpenFunction:
        case Column::CANOpenNode:
        case Column::Direction:
        case Column::Extended:
        case Column::Bus:
        case Column::Remote:
        case Column::Length:
            return Qt::AlignHCenter;
        default:
            return Qt::AlignLeft;
        }
    }

    if (role == Qt::TextColorRole)
    {
        if (dbcHandler != nullptr && interpretFrames)
        {
            DBC_MESSAGE *msg = dbcHandler->findMessage(thisFrame);
            if (msg != nullptr)
            {
                return msg->fgColor;
            }
        }
        return QApplication::palette().color(QPalette::WindowText);
    }

    if (role == Qt::DisplayRole) {
        switch (Column(index.column()))
        {
        case Column::TimeStamp:            
            //Reformatting the output a bit with custom code
            if (overwriteDups)
            {
                if (timeSeconds) return QString::number(thisFrame.timedelta / 1000000.0, 'f', 5);
                return QString::number(thisFrame.timedelta);
            }
            else ts = Utility::formatTimestamp(thisFrame.timeStamp().microSeconds());
            if (ts.type() == QVariant::Double) return QString::number(ts.toDouble(), 'f', 5); //never scientific notation, 5 decimal places
            if (ts.type() == QVariant::LongLong) return QString::number(ts.toLongLong()); //never scientific notion, all digits shown
            if (ts.type() == QVariant::DateTime) return ts.toDateTime().toString(timeFormat); //custom set format for dates and times
            return Utility::formatTimestamp(thisFrame.timeStamp().microSeconds());
        case Column::FrameId:
            return Utility::formatCANID(thisFrame.frameId(), thisFrame.hasExtendedFrameFormat());
        case Column::Extended:
            return QString::number(thisFrame.hasExtendedFrameFormat());
        case Column::Remote:
            if (!overwriteDups) return QString::number(thisFrame.frameType() == QCanBusFrame::RemoteRequestFrame);
            return QString::number(thisFrame.frameCount);
        case Column::Direction:
            if (thisFrame.isReceived) return QString(tr("Rx"));
            return QString(tr("Tx"));
        case Column::Bus:
            return QString::number(thisFrame.bus);
        case Column::Length:
            return QString::number(dataLen);
        case Column::ASCII:
            if (thisFrame.frameId() >= 0x7FFFFFF0ull)
            {
                tempString.append("MARK ");
                tempString.append(QString::number(thisFrame.frameId() & 0x7));
                return tempString;
            }
            if (thisFrame.frameType() == QCanBusFrame::RemoteRequestFrame)
            {
                tempString = "Remote request frame";
            }
            else
            {
                int sdo = Utility::isSDO(thisFrame.frameId());
                tempString = printSDO(sdo, data);
            }
            return tempString;
        case Column::Data:
            if (dataLen < 0) dataLen = 0;
            //if (useHexMode) tempString.append("0x ");
            if (thisFrame.frameType() == QCanBusFrame::RemoteRequestFrame) {
                return tempString;
            }
            for (int i = 0; i < dataLen; i++)
            {
                if (useHexMode) tempString.append( QString::number(data[i], 16).toUpper().rightJustified(2, '0'));
                else tempString.append(QString::number(data[i], 10));
                tempString.append(" ");
            }
            //now, if we're supposed to interpret the data and the DBC handler is loaded then use it
            if (dbcHandler != nullptr && interpretFrames)
            {
                DBC_MESSAGE *msg = dbcHandler->findMessage(thisFrame);
                if (msg != nullptr)
                {
                    tempString.append("   <" + msg->name + ">\n");
                    if (msg->comment.length() > 1) tempString.append(msg->comment + "\n");
                    for (int j = 0; j < msg->sigHandler->getCount(); j++)
                    {
                        QString sigString;
                        DBC_SIGNAL* sig = msg->sigHandler->findSignalByIdx(j);
                        if (sig->processAsText(thisFrame, sigString))
                        {
                            tempString.append(sigString);
                            tempString.append("\n");
                        }
                        else if (sig->isMultiplexed && overwriteDups) //wasn't in this exact frame but is in the message. Use cached value
                        {
                            tempString.append(sig->makePrettyOutput(sig->cachedValue.toDouble(), sig->cachedValue.toLongLong()));
                            tempString.append("\n");
                        }
                    }
                }
            }
//            tempString = "<html><body><p><i>This text is italic</i></p></body></html>";
//            tempString = tempString.toHtmlEscaped();
            return tempString;
        case Column::CANOpenFunction:
            return Utility::formatCANOpenFunction(thisFrame.frameId(), thisFrame.hasExtendedFrameFormat());
        case Column::CANOpenNode:
            return Utility::formatCANOpenNode(thisFrame.frameId(), thisFrame.hasExtendedFrameFormat());
        default:
            return tempString;
        }
    }

    return QVariant();
}

QVariant CANFrameModel::headerData(int section, Qt::Orientation orientation,
                                     int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal)
    {
        switch (Column(section))
        {
        case Column::TimeStamp:
            if (overwriteDups) return QString(tr("Time Delta"));
            return QString(tr("Timestamp"));
        case Column::FrameId:
            return QString(tr("COB-ID"));
        case Column::Extended:
            return QString(tr("Ext"));
        case Column::Remote:
            if (!overwriteDups) return QString(tr("RTR"));
            return QString(tr("Cnt"));
        case Column::Direction:
            return QString(tr("Dir"));
        case Column::Bus:
            return QString(tr("Bus"));
        case Column::Length:
            return QString(tr("Len"));
        case Column::ASCII:
            return QString(tr("ASCII"));
        case Column::Data:
            return QString(tr("Data"));
        case Column::CANOpenFunction:
            return QString(tr("Func"));
        case Column::CANOpenNode:
            return QString(tr("Node"));
        default:
            return QString("");
        }
    }
    else
        return QString::number(section + 1);

    return QVariant();
}

bool CANFrameModel::any_filters_are_configured(void)
{
    for (auto const &val : filters)
    {
        if (val == true)
            continue;
        else
            return true;
    }
    return false;
}


void CANFrameModel::addFrame(const CANFrame& frame, bool autoRefresh = false)
{
    /*TODO: remove mutex */
    mutex.lock();
    CANFrame tempFrame;
    tempFrame = frame;

    tempFrame.setTimeStamp(QCanBusFrame::TimeStamp(0, tempFrame.timeStamp().microSeconds() - timeOffset));

    lastUpdateNumFrames++;

    //if this ID isn't found in the filters list then add it and show it by default
    if (!filters.contains(tempFrame.frameId() & 0x7F))
    {
        // if there are any filters already configured, leave the new filter disabled
        if (any_filters_are_configured())
            filters.insert(tempFrame.frameId() & 0x7F, false);
        else
            filters.insert(tempFrame.frameId() & 0x7F, true);
        needFilterRefresh = true;
    }

    if (!overwriteDups)
    {
        frames.append(tempFrame);
        if (filters[tempFrame.frameId() & 0x7F] && filterFrameConsideringFunction(tempFrame.frameId()))
        {
            if (autoRefresh) beginInsertRows(QModelIndex(), filteredFrames.count(), filteredFrames.count());
            tempFrame.frameCount = 1;
            filteredFrames.append(tempFrame);
            if (autoRefresh) endInsertRows();
        }
    }
    else //yes, overwrite dups
    {
        bool found = false;
        for (int i = 0; i < frames.count(); i++)
        {
            if ( (frames[i].frameId() == tempFrame.frameId()) && (frames[i].bus == tempFrame.bus) )
            {
                tempFrame.frameCount = frames[i].frameCount + 1;
                tempFrame.timedelta = tempFrame.timeStamp().microSeconds() - frames[i].timeStamp().microSeconds();
                frames.replace(i, tempFrame);
                found = true;
                break;
            }
        }
        if (!found)
        {
            frames.append(tempFrame);
            if (filters[tempFrame.frameId() & 0x7F] && filterFrameConsideringFunction(tempFrame.frameId()))
            {
                if (autoRefresh) beginInsertRows(QModelIndex(), filteredFrames.count(), filteredFrames.count());
                tempFrame.frameCount = 1;
                tempFrame.timedelta = 0;
                filteredFrames.append(tempFrame);
                if (autoRefresh) endInsertRows();
            }
        }
        else
        {
            for (int j = 0; j < filteredFrames.count(); j++)
            {
                if ( (filteredFrames[j].frameId() == tempFrame.frameId()) && (filteredFrames[j].bus == tempFrame.bus) )
                {
                    if (autoRefresh) beginResetModel();
                    filteredFrames.replace(j, tempFrame);
                    if (autoRefresh) endResetModel();
                }
            }
        }
    }

    mutex.unlock();
}


void CANFrameModel::addFrames(const CANConnection*, const QVector<CANFrame>& pFrames)
{
    foreach(const CANFrame& frame, pFrames)
    {
        addFrame(frame);
    }
    if (overwriteDups) //if in overwrite mode we'll update every time frames come in
    {
        beginResetModel();
        endResetModel();
    }
}

void CANFrameModel::sendRefresh()
{
    qDebug() << "Sending mass refresh";
    QVector<CANFrame> tempContainer;
    int count = frames.count();
    for (int i = 0; i < count; i++)
    {
        if (filters[frames[i].frameId() & 0x7F] && filterFrameConsideringFunction(frames[i].frameId()))
        {
            tempContainer.append(frames[i]);
        }
    }
    mutex.lock();
    beginResetModel();
    filteredFrames.clear();
    filteredFrames.reserve(preallocSize);
    filteredFrames.append(tempContainer);

    lastUpdateNumFrames = 0;
    endResetModel();
    mutex.unlock();
}

void CANFrameModel::sendRefresh(int pos)
{
    beginInsertRows(QModelIndex(), pos, pos);
    endInsertRows();
}

//issue a refresh for the last num entries in the model.
//used by the serial worker to do batch updates so it doesn't
//have to send thousands of messages per second
int CANFrameModel::sendBulkRefresh()
{
    //int num = filteredFrames.count() - lastUpdateNumFrames;
    if (lastUpdateNumFrames <= 0) return 0;

    if (lastUpdateNumFrames == 0 && !overwriteDups) return 0;
    if (filteredFrames.count() == 0) return 0;

    qDebug() << "Bulk refresh of " << lastUpdateNumFrames;

    beginResetModel();
    endResetModel();

    int num = lastUpdateNumFrames;
    lastUpdateNumFrames = 0;

    return num;
}

void CANFrameModel::clearFrames()
{
    mutex.lock();
    this->beginResetModel();
    frames.clear();
    filteredFrames.clear();
//    filters.clear();
    frames.reserve(preallocSize);
    filteredFrames.reserve(preallocSize);
    this->endResetModel();
    lastUpdateNumFrames = 0;
    mutex.unlock();

    emit updatedFiltersList();
}

/*
 * Since the getListReference function returns readonly
 * you can't insert frames with it. Instead this function
 * allows for a mass import of frames into the model
 */
void CANFrameModel::insertFrames(const QVector<CANFrame> &newFrames)
{
    //not resetting the model here because the serial worker automatically does a bulk refresh every 1/4 second
    //and that refresh will cause the view to update. If you do both it usually ends up thinking you have
    //double the number of frames.
    //beginResetModel();
    mutex.lock();
    int insertedFiltered = 0;
    for (int i = 0; i < newFrames.count(); i++)
    {
        frames.append(newFrames[i]);
        if (!filters.contains(newFrames[i].frameId() & 0x7F))
        {
            filters.insert(newFrames[i].frameId() & 0x7F, true);
            needFilterRefresh = true;
        }
        if (filters[newFrames[i].frameId() & 0x7F] && filterFrameConsideringFunction(newFrames[i].frameId()))
        {
            insertedFiltered++;
            filteredFrames.append(newFrames[i]);
        }
    }
    lastUpdateNumFrames = newFrames.count();
    mutex.unlock();
    //endResetModel();
    //beginInsertRows(QModelIndex(), filteredFrames.count() + 1, filteredFrames.count() + insertedFiltered);
    //endInsertRows();
    if (needFilterRefresh) emit updatedFiltersList();
}

int CANFrameModel::getIndexFromTimeID(unsigned int ID, double timestamp)
{
    int bestIndex = -1;
    int64_t intTimeStamp = static_cast<int64_t> (timestamp * 1000000l);
    for (int i = 0; i < frames.count(); i++)
    {
        if ((frames[i].frameId() == ID))
        {
            if (frames[i].timeStamp().microSeconds() <= intTimeStamp) bestIndex = i;
            else break; //drop out of loop as soon as we pass the proper timestamp
        }
    }
    return bestIndex;
}

void CANFrameModel::loadFilterFile(QString filename)
{
    QFile *inFile = new QFile(filename);
    QByteArray line;
    int ID;

    if (!inFile->open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    filters.clear();

    while (!inFile->atEnd()) {
        line = inFile->readLine().simplified();
        if (line.length() > 2)
        {
            QList<QByteArray> tokens = line.split(',');
            ID = tokens[0].toInt(nullptr, 16);
            if (tokens[1].toUpper() == "T") filters.insert(ID & 0x7F, true);
                else filters.insert(ID & 0x7F, false);
        }
    }
    inFile->close();

    sendRefresh();

    emit updatedFiltersList();
}

void CANFrameModel::saveFilterFile(QString filename)
{
    QFile *outFile = new QFile(filename);

    if (!outFile->open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QMap<int, bool>::const_iterator it;
    for (it = filters.begin(); it != filters.end(); ++it)
    {
        outFile->write(QString::number(it.key(), 16).toUtf8());
        outFile->putChar(',');
        if (it.value()) outFile->putChar('T');
            else outFile->putChar('F');
        outFile->write("\n");
    }
    outFile->close();
}

bool CANFrameModel::needsFilterRefresh()
{
    bool temp = needFilterRefresh;
    needFilterRefresh = false;
    return temp;
}

/*
 *This used to not be const correct but it is now. So, there's little harm in
 * allowing external code to peek at our frames. There's just no touching.
 * This ability to get a direct read-only reference speeds up a variety of
 * external code that needs to access frames directly and doesn't care about
 * this model's normal output mechanism.
 */
const QVector<CANFrame>* CANFrameModel::getListReference() const
{
    return &frames;
}

const QVector<CANFrame>* CANFrameModel::getFilteredListReference() const
{
    return &filteredFrames;
}

const QMap<int, bool>* CANFrameModel::getFiltersReference() const
{
//    printf("******** %d\n", this->filterNMTon);
//    fflush(stdout);
    return &filters;
}
