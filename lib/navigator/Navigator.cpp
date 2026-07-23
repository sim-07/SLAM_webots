#include <string.h>
#include <math.h>
#include <float.h>
#include <set>

#include "Navigator.h"
#include <stack>
#include <map>
#include <iostream>
#include <algorithm>

using namespace std;

Navigator::Navigator()
{
    _currPos = {0, 0};
    _currDir = 0;
}

double Navigator::getDir()
{
    return _currDir;
}

Pos Navigator::calcFinalCell(Pos start, double angle, float dis)
{
    int16_t dX = std::round((std::cos(angle) * dis) / CELL_CM);
    int16_t dY = std::round((std::sin(angle) * dis) / CELL_CM);

    return {static_cast<int16_t>(start.x + dX), static_cast<int16_t>(start.y + dY)};
}

void Navigator::updateCompass(const double *compassValues)
{
    if (std::isnan(compassValues[0]))
        return;

    double rawCompassRad = -std::atan2(compassValues[0], compassValues[2]);

    if (!_isSensorsCalibrated)
    {

        _compassOffset = -rawCompassRad; // Angolo iniziale nella mia logica è 0, quindi metto negativo (0-rawCompassRad)

        std::cout << "[COMPASS RAW] X: " << compassValues[0]
                  << " | Y: " << compassValues[1]
                  << " | Z: " << compassValues[2] << std::endl;

        std::cout << "[CALIBRATION] Compass Offset (rad): " << _compassOffset << std::endl;

        _isSensorsCalibrated = true;
    }

    double correctedCompassRad = rawCompassRad + _compassOffset;

    _currDirCompass = std::atan2(std::sin(correctedCompassRad), std::cos(correctedCompassRad));
}

void Navigator::setGyro(Gyroscope *gyro)
{
    _gyro = gyro;
}

void Navigator::setDir(double rad)
{
    //std::cout << "_currDirCompass WETBOTS: " << _currDirCompass << std::endl;

    if (_gyro != nullptr)
    {
        //std::cout << "_gyro WETBOTS: " << _gyro->getCurrAngle() << std::endl;
    }

    //std::cout << "_currDir calc from encoders: " << rad << std::endl;
    _currDir = _currDirCompass;
}

void Navigator::setCurrPos(Pos newPos)
{
    // std::cout << "Current position in WEBOTS set to: " << _currXWebots << ":" << _currYWebots << std::endl;

    //std::cout << "Current position in nav set to: " << newPos.x << ":" << newPos.y << std::endl;
    _currPos = newPos;
}

const std::map<Pos, Chunk> &Navigator::getMap() const
{
    return _map;
}

Pos Navigator::getChunkPos(Pos p)
{
    return {(int16_t)(p.x >> 4), (int16_t)(p.y >> 4)}; // / 16 e arrotondato per difetto, mantenendo il segno
}

int16_t Navigator::getPosIndex(Pos p)
{
    int16_t posCellX = p.x & 0x0F; // % 16 sempre positivo
    int16_t posCellY = p.y & 0x0F;

    return (posCellY * CHUNK_WIDTH) + posCellX;
}

void Navigator::sculpt(int16_t targetX, int16_t targetY, SensorType st)
{

    int v = 0;
    switch (st)
    {
    case ULTRASONIC:
        v = ULTRASONIC_A;
        break;

    case LASER:
        v = LASER_A;
        break;

    default:
        break;
    }

    createBlanks({targetX, targetY});

    float dis = (calcDistanceBetween(getPos(), {targetX, targetY}) / 10); // Serve per evitare che la posizione attuale venga impostata come ostacolo
    if (dis > PADDING_SCULPT)
    {
        int xMax = targetX + PADDING_SCULPT;
        int xMin = targetX - PADDING_SCULPT;
        int yMax = targetY + PADDING_SCULPT;
        int yMin = targetY - PADDING_SCULPT;

        for (int16_t i = xMin; i <= xMax; i++)
        {
            for (int16_t j = yMin; j <= yMax; j++)
            {
                Pos cPos = getChunkPos({i, j});
                int16_t cellIndex = getPosIndex({i, j});

                Chunk &currChunk = _map[cPos];

                if (currChunk.cells[cellIndex] - v <= 0)
                {
                    currChunk.cells[cellIndex] = 0;
                }
                else// if (currChunk.cells[cellIndex] > THRESHOLD_OBSTACLE)
                {
                    currChunk.cells[cellIndex] -= v;
                }
            }
        }
    }
}

Route Navigator::calcRoute(Pos dest)
{
    std::cout << "Inside calcRoute in nav for: " << dest.x << ":" << dest.y << std::endl;

    return aStar(_currPos, dest);
}

void Navigator::createBlanks(Pos target)
{

    Pos chunkCurrPos = getChunkPos({getPos().x, getPos().y});
    Pos chunkDestPos = getChunkPos(target);
    int16_t cellIndex = getPosIndex(target);

    int16_t x0 = getPos().x;
    int16_t y0 = getPos().y;
    int16_t x1 = target.x;
    int16_t y1 = target.y;

    // delta x e y
    int16_t dX = std::abs(x1 - x0);
    int16_t dY = -std::abs(y1 - y0);

    if (dX > 1000 || dY < -1000)
    {
        std::cout << "Map too big " << std::endl;
        return;
    }

    // direzione destinazione
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;

    int16_t err = dX + dY;

    Pos lastPos = {0, 0};
    Chunk *currChunk = nullptr;

    while (true)
    {
        // esco prima della destinazione, non voglio resettare l'ultima cella
        if (x0 == x1 && y0 == y1)
            break;

        // int16_t pX[] = {-1, 1, 0, 0};
        // int16_t pY[] = {0, 0, 1, -1};
        // // Abbasso il valore celle celle in linea retta con la destinazione e quelle adiacenti ad esse
        // for (int16_t i = 0; i < 4; i++)
        // {
        //     float dis = (calcDistanceBetween({x0 + pX[i], y0 + pY[i]}, {x1, y1}) / 10);
        //     // Calcolo la distanza tra la cella attualmente analizzata e la destinazione, se è < del padding mi fermo per non sovrascriverlo
        //     if (dis <= PADDING_SCULPT) {
        //         break;
        //     }

        //     Pos cPos = getChunkPos({x0 + pX[i], y0 + pY[i]});
        //     if (currChunk == nullptr || cPos.x != lastPos.x || cPos.y != lastPos.y)
        //     {
        //         // riprendo il chunk dal map solo se è cambiato o è nullptr
        //         currChunk = &_map[cPos];
        //         lastPos = cPos;
        //     }

        //     int16_t cellIndex = getPosIndex({x0 + pX[i], y0 + pY[i]});
        //     if (cellIndex < CHUNK_DIM && currChunk->cells[cellIndex] <= DEFAULT_VAL)
        //     {
        //         if (currChunk->cells[cellIndex] + BLANK_A > 255)
        //         {
        //             currChunk->cells[cellIndex] = 255;
        //         }
        //         else if (currChunk->cells[cellIndex] > THRESHOLD_OBSTACLE)
        //         {
        //             currChunk->cells[cellIndex] += BLANK_A;
        //         }
        //         else
        //         {
        //             currChunk->cells[cellIndex] += std::floor(BLANK_A / 2);
        //         }
        //     }
        // }

        Pos cPos = getChunkPos({x0, y0});
        if (currChunk == nullptr || cPos.x != lastPos.x || cPos.y != lastPos.y)
        {
            // riprendo il chunk dal map solo se è cambiato o è nullptr
            currChunk = &_map[cPos];
            lastPos = cPos;
        }

        int16_t cellIndex = getPosIndex({x0, y0});
        if (cellIndex < CHUNK_DIM && currChunk->cells[cellIndex] <= DEFAULT_VAL)
        {
            float dis = (calcDistanceBetween({x0, y0}, {x1, y1}) / 10);
            // Calcolo la distanza tra la cella attualmente analizzata e la destinazione, se è < del padding mi fermo per non sovrascriverlo
            if (dis <= PADDING_SCULPT)
            {
                break;
            }

            if (currChunk->cells[cellIndex] + BLANK_A > 255)
            {
                currChunk->cells[cellIndex] = 255;
            }
            else
            {
                currChunk->cells[cellIndex] += BLANK_A;
            }
        }

        int e2 = 2 * err;

        if (e2 >= dY)
        {
            err += dY;
            x0 += sx;
        }

        if (e2 <= dX)
        {
            err += dX;
            y0 += sy;
        }
    }
}

///////////////////////// A* /////////////////////////

bool Navigator::isFree(int16_t x, int16_t y)
{
    Pos p = getChunkPos({x, y});

    auto it = _map.find(p);

    if (it == _map.end())
    {
        return false;
    }

    int16_t cellIndex = getPosIndex({x, y});
    bool free = it->second.cells[cellIndex] > DEFAULT_VAL;

    return free;
}

bool isDestination(int16_t row, int16_t col, Pos dest)
{
    if (row == dest.x && col == dest.y)
        return true;
    else
        return false;
}

int Navigator::calcDistanceBetween(Pos start, Pos dest)
{
    // return (sqrt(
    //     (start.x - dest.x) * (start.x - dest.x) + (start.y - dest.y) * (start.y - dest.y)));

    int dX = std::abs(start.x - dest.x);
    int dY = std::abs(start.y - dest.y);

    if (dX > dY)
    {
        return 14 * dY + 10 * (dX - dY);
    }
    else
    {
        return 14 * dX + 10 * (dY - dX);
    }
}

Route tracePath(std::map<Pos, Node> &cellDetails, Pos dest)
{
    Route r;

    r.numSteps = 0;

    int16_t row = dest.x;
    int16_t col = dest.y;

    while (true)
    {
        auto it = cellDetails.find({row, col});

        if (it == cellDetails.end())
            break;

        Pos p = {row, col};
        r.route.push_back(p);
        r.numSteps++;

        if (it->second.parent_i == row && it->second.parent_j == col) // la cella iniziale è l'unica ad avere i parents con le stesse proprie coordinate
        {
            break;
        }

        row = it->second.parent_i;
        col = it->second.parent_j;
    }

    std::reverse(r.route.begin(), r.route.end());

    return r;
}

const int MAX_V = 65535;
Route Navigator::aStar(Pos start, Pos goal)
{

    // cella di partenza
    int16_t currRow = start.x;
    int16_t currCol = start.y;

    if (start.x == goal.x && start.y == goal.y)
    {
        std::cout << "Return empty route, goal == start in aStar" << std::endl;
        return Route();
    }

    std::map<Pos, Node> cellDetails;                 // informazioni sulle celle visitate (coordinate, g, f, h, parent)
    set<pair<int, pair<int16_t, int16_t>>> openList; // celle da esplorare ordinate in base alla f
    std::set<Pos> closedList;                        // celle già esplorate

    openList.insert({0, {currRow, currCol}}); // fNew, {x, y}
    cellDetails[{currRow, currCol}] = {currRow, currCol, 0, 0, 0};

    int8_t dX[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int8_t dY[] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!openList.empty())
    {
        auto p = *openList.begin();
        openList.erase(openList.begin());

        currRow = p.second.first;  // x
        currCol = p.second.second; // y
        closedList.insert({currRow, currCol});

        for (int k = 0; k < 8; k++)
        {
            // vengono controllate tutte le 8 direzioni
            int16_t nRow = currRow + dX[k];
            int16_t nCol = currCol + dY[k];

            auto it = cellDetails.find({nRow, nCol});

            if (isDestination(nRow, nCol, {goal.x, goal.y}))
            {
                // se la prossima cella (nRow e nCol) è destinazione, metto i parent della prossima cella con i valori row e col della cella corrente e return
                cellDetails[{nRow, nCol}].parent_i = currRow;
                cellDetails[{nRow, nCol}].parent_j = currCol;

                return tracePath(cellDetails, {nRow, nCol});
            }

            if (!closedList.count({nRow, nCol}) && isFree(nRow, nCol))
            {
                // se non è la meta controllo che non sia già stato visto e che non sia un ostacolo
                int cost = (k < 4) ? 10 : 14; // se è diagonale costerà 1.41, gli ultimi 4 k sono diagonali
                int gNew = cellDetails[{currRow, currCol}].g + cost;
                int hNew = calcDistanceBetween({nRow, nCol}, {goal.x, goal.y}) * 10;
                int fNew = gNew + hNew;

                if (it == cellDetails.end() || it->second.f > fNew)
                {
                    cellDetails[{nRow, nCol}] = {
                        currRow,
                        currCol,
                        fNew,
                        gNew,
                        hNew};

                    openList.insert({fNew, {nRow, nCol}}); // aggiungo il prossimo valore alla lista delle celle da esplorare
                }
            }
        }
    }

    return Route();
}