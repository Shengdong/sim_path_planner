#ifndef GRID2D_H
#define GRID2D_H

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <inttypes.h>
#include <vector>


template<typename T>
class Grid2D
{
public:
/**
 * Constructor. The <code> defaultValue </code> parameter is optional.
 * @param resolution side length of a single square grid cell.
 * @param rows number of rows.
 * @param cols number of columns.
 * @param defaultValue optional custom default value for template type T.
 */
    Grid2D(double resolution, int rows, int cols,
           const T& defaultValue = T());

/**
 * Default destructor. Non-virtual as Grid is not meant to be inherited.
 */
    ~Grid2D();

/**
 * Copy constructor.
 */
    Grid2D(const Grid2D<T>& other);

/**
 * Copy assignment operator.
 */
    Grid2D<T>& operator=(const Grid2D<T>& other);

/**
 * Resets all the grid cells to the default value.
 */
    void reset();

/**
 * Moves the grid to be centered at <code>(x, y)</code>.
 * @param x new center x-coordinate.
 * @param y new center y-coordinate.
 */
    bool recenterXY(double x, double y);

/**
 * Crops the grid to the smallest possible size while excluding cells
 * with default values.
 * @param enforceSquareBoundary If true, new grid dimensions are equal.
 */
    void crop(bool enforceSquareBoundary = false);

/**
 * Get the coordinates of the center of the grid.
 */
    void getCenterXY(double& x, double& y) const;

    bool recenterMapRC(int r, int c);

/**
 * Get the cell located at row-column coordinates <code>(r,c)</code>
 * @param r     row-coordinate
 * @param c     column-coordinate
 */
    const T* getLocalRC(int r, int c) const;
    const T* getMapRC(int r, int c) const;
    const T* getXY(double x, double y) const;

    T* getLocalRC(int r, int c);
    T* getMapRC(int r, int c);
    T* getXY(double x, double y);

    bool convertXYtoLocalRC(double x, double y,
                            int& r, int& c) const;

    bool convertXYtoMapRC(double x, double y,
                          int& r, int& c) const;

    bool convertLocalRCtoXY(int r, int c,
                            double& x, double& y) const;

    bool convertMapRCtoXY(int r, int c,
                          double& x, double& y) const;

    T* getRawCopy(void);

    double resolution(void) const
    {
        return m_resolution;
    }
    int rows(void) const
    {
        return m_rows;
    }
    int cols(void) const
    {
        return m_cols;
    }

    int mapRowStart(void) const
    {
        return m_mapR0;
    }
    int mapColStart(void) const
    {
        return m_mapC0;
    }

    int arrayRowStart(void) const
    {
        return m_arrayR0;
    }
    int arrayColStart(void) const
    {
        return m_arrayC0;
    }

    int& mapRowStart(void)
    {
        return m_mapR0;
    }
    int& mapColStart(void)
    {
        return m_mapC0;
    }

    int& arrayRowStart(void)
    {
        return m_arrayR0;
    }
    int& arrayColStart(void)
    {
        return m_arrayC0;
    }

private:
    void addColumnEast(void);
    void addColumnWest(void);
    void addRowNorth(void);
    void addRowSouth(void);

    bool emptyRow(int r) const;
    bool emptyColumn(int c) const;

    double m_resolution; /**< map resolution in meters */
    int m_rows, m_cols; /**< size of grid */
    int m_mapR0, m_mapC0; /**< grid coordinates of lower left corner of map */
    int m_arrayR0, m_arrayC0; /**< position of lower left corner in array */
    T m_defaultValue; /**< default value of map cell */

    /// actual map data
    std::vector<T> m_cells;
};


template<typename T>
Grid2D<T>::Grid2D(double resolution, int rows, int cols,
                  const T& defaultValue)
 : m_resolution(resolution)
 , m_rows(rows), m_cols(cols)
 , m_mapR0(0), m_mapC0(0)
 , m_arrayR0(0), m_arrayC0(0)
 , m_defaultValue(defaultValue)
{
    m_cells.resize(m_rows * m_cols);
    for (int i = 0; i < m_rows * m_cols; i++)
    {
        m_cells.at(i) = m_defaultValue;
    }
}

template<typename T>
Grid2D<T>::~Grid2D()
{

}

template<typename T>
Grid2D<T>::Grid2D(const Grid2D<T>& other)
 : m_resolution(other.m_resolution)
 , m_rows(other.m_rows), m_cols(other.m_cols)
 , m_mapR0(other.m_mapR0), m_mapC0(other.m_mapC0)
 , m_arrayR0(other.m_arrayR0), m_arrayC0(other.m_arrayC0)
 , m_defaultValue(other.m_defaultValue)
 , m_cells(other.m_cells)
{

}

template<typename T>
Grid2D<T>& Grid2D<T>::operator=(const Grid2D<T>& other)
{
    // check for self-assignment
    if (this != &other)
    {
        this->m_resolution = other.m_resolution;
        this->m_rows = other.m_rows;
        this->m_cols = other.m_cols;
        this->m_mapR0 = other.m_mapR0;
        this->m_mapC0 = other.m_mapC0;
        this->m_arrayR0 = other.m_arrayR0;
        this->m_arrayC0 = other.m_arrayC0;
        this->m_defaultValue = other.m_defaultValue;
        this->m_cells = other.m_cells;
    }

    return (*this);
}

template<typename T>
void Grid2D<T>::reset()
{
    for (int i = 0; i < m_rows * m_cols; i++)
    {
        m_cells.at(i) = m_defaultValue;
    }
}

template<typename T>
bool Grid2D<T>::recenterXY(double x, double y)
{
    return recenterMapRC(floor(y / m_resolution), floor(x / m_resolution));
}

template<typename T>
void Grid2D<T>::crop(bool enforceSquareBoundary)
{
    int rLow = 0, rHigh = m_rows - 1;
    int cLow = 0, cHigh = m_cols - 1;

    // find smallest possible boundary
    for (int r = 0; r < m_rows; ++r)
    {
        if (!emptyRow(r))
        {
            break;
        }
        ++rLow;
    }

    for (int r = m_rows - 1; r >= 0; --r)
    {
        if (!emptyRow(r))
        {
            break;
        }
        --rHigh;
    }

    for (int c = 0; c < m_cols; ++c)
    {
        if (!emptyColumn(c))
        {
            break;
        }
        ++cLow;
    }

    for (int c = m_cols - 1; c >= 0; --c)
    {
        if (!emptyColumn(c))
        {
            break;
        }
        --cHigh;
    }

    if (rLow > rHigh)
    {
        rLow = rHigh = 0;
    }
    if (cLow > cHigh)
    {
        cLow = cHigh = 0;
    }

    // normalize number of cells spanning width/height to be an odd number
    if ((rHigh - rLow + 1) % 2 == 0)
    {
        ++rHigh;
    }

    if ((cHigh - cLow + 1) % 2 == 0)
    {
        ++cHigh;
    }

    if (enforceSquareBoundary)
    {
        int heightInCells = rHigh - rLow + 1;
        int widthInCells = cHigh - cLow + 1;

        if (heightInCells > widthInCells)
        {
            int diff = heightInCells - widthInCells;
            cLow -= diff / 2;
            cHigh += diff / 2;
        }
        else if (widthInCells > heightInCells)
        {
            int diff = widthInCells - heightInCells;
            rLow -= diff / 2;
            rHigh += diff / 2;
        }
    }

    Grid2D<T> gridCropped(m_resolution, rHigh - rLow + 1, cHigh - cLow + 1,
                          m_defaultValue);

    double xCenter, yCenter;
    convertLocalRCtoXY((rHigh - rLow) / 2 + rLow, (cHigh - cLow) / 2 + cLow,
                       xCenter, yCenter);
    gridCropped.recenterXY(xCenter, yCenter);

    for (int r = rLow; r <= rHigh; ++r)
    {
        for (int c = cLow; c <= cHigh; ++c)
        {
            double x, y;
            convertLocalRCtoXY(r, c, x, y);

            *(gridCropped.getXY(x,y)) = *(getLocalRC(r,c));
        }
    }

    *this = gridCropped;
}

template<typename T>
void Grid2D<T>::getCenterXY(double& x, double& y) const
{
    x = (static_cast<double>(m_mapR0) + m_rows / 2.0) * m_resolution;
    y = (static_cast<double>(m_mapC0) + m_cols / 2.0) * m_resolution;
}

template<typename T>
bool Grid2D<T>::recenterMapRC(int r, int c)
{
    int corner_r = r - m_rows / 2;
    int corner_c = c - m_cols / 2;

    int dr = corner_r - m_mapR0;
    int dc = corner_c - m_mapC0;

    if (dr == 0 && dc == 0)
    {
        return false;
    }

    if (abs(dr) >= m_rows || abs(dc) >= m_cols)
    {
        reset();
        m_mapR0 = corner_r;
        m_mapC0 = corner_c;
        m_arrayR0 = 0;
        m_arrayC0 = 0;
    }
    else {
        if (dr > 0)
        {
            for (int i = 0; i < dr; i++)
            {
                addRowNorth();
            }
        }
        else if (dr < 0)
        {
            for (int i = 0; i < abs(dr); i++)
            {
                addRowSouth();
            }
        }
        if (dc > 0)
        {
            for (int i = 0; i < dc; i++)
            {
                addColumnEast();
            }
        }
        else if (dc < 0)
        {
            for (int i = 0; i < abs(dc); i++)
            {
                addColumnWest();
            }
        }
    }
    return true;
}

template<typename T>
const T* Grid2D<T>::getLocalRC(int r, int c) const
{
    if (r < 0 || c < 0 || r >= m_rows || c >= m_cols)
    {
        return NULL;
    }

    r = (r + m_arrayR0) % m_rows;
    c = (c + m_arrayC0) % m_cols;

    const T& cell = m_cells.at(r * m_cols + c);

    return &cell;
}

template<typename T>
const T* Grid2D<T>::getMapRC(int r, int c) const
{
    return getLocalRC(r - m_mapR0, c - m_mapC0);
}

template<typename T>
const T* Grid2D<T>::getXY(double x, double y) const
{
    int r = static_cast<int>(floor(y / m_resolution));
    int c = static_cast<int>(floor(x / m_resolution));

    return getMapRC(r, c);
}

template<typename T>
T* Grid2D<T>::getLocalRC(int r, int c)
{
    return const_cast<T*>(
            static_cast<const Grid2D<T>&>(*this).getLocalRC(r, c));
}

template<typename T>
T* Grid2D<T>::getMapRC(int r, int c)
{
    return const_cast<T*>(
            static_cast<const Grid2D<T>&>(*this).getMapRC(r, c));
}

template<typename T>
T* Grid2D<T>::getXY(double x, double y)
{
    return const_cast<T*>(
        static_cast<const Grid2D<T>&>(*this).getXY(x, y));
}

template<typename T>
bool Grid2D<T>::convertXYtoLocalRC(double x, double y,
                                   int& r, int& c) const
{
    r = static_cast<int>(floor(y / m_resolution)) - m_mapR0;
    c = static_cast<int>(floor(x / m_resolution)) - m_mapC0;

    if (r < 0 || c < 0 || r >= m_rows || c >= m_cols)
    {
        return false;
    }

    return true;
}

template<typename T>
bool Grid2D<T>::convertXYtoMapRC(double x, double y,
                                 int& r, int& c) const
{
    int rLocal, cLocal;

    if (!convertXYtoLocalRC(x, y, rLocal, cLocal))
    {
        return false;
    }

    r = rLocal + m_mapR0;
    c = cLocal + m_mapC0;

    return true;
}

template<typename T>
bool Grid2D<T>::convertLocalRCtoXY(int r, int c,
                                   double& x, double& y) const
{
    if (r < 0 || c < 0 || r >= m_rows || c >= m_cols)
    {
        return false;
    }

    y = (static_cast<double>(m_mapR0 + r) + 0.5) * m_resolution;
    x = (static_cast<double>(m_mapC0 + c) + 0.5) * m_resolution;

    return true;
}

template<typename T>
bool Grid2D<T>::convertMapRCtoXY(int r, int c,
                                 double& x, double& y) const
{
    return convertLocalRCtoXY(r - m_mapR0, c - m_mapC0, x, y);
}

template<typename T>
T* Grid2D<T>::getRawCopy(void)
{
    return &(m_cells.at(0));
}

template<typename T>
void Grid2D<T>::addColumnEast(void)
{
    for (int r = 0; r < m_rows; r++)
    {
        m_cells.at(r * m_cols + m_arrayC0) = m_defaultValue;
    }

    m_mapC0++;
    m_arrayC0++;
    if (m_arrayC0 == m_cols)
    {
        m_arrayC0 = 0;
    }
}

template<typename T>
void Grid2D<T>::addColumnWest(void)
{
    m_mapC0--;
    if (m_arrayC0 != 0)
    {
        m_arrayC0--;
    }
    else
    {
        m_arrayC0 = m_cols - 1;
    }

    for (int r = 0; r < m_rows; r++)
    {
        m_cells.at(r * m_cols + m_arrayC0) = m_defaultValue;
    }
}

template<typename T>
void Grid2D<T>::addRowNorth(void)
{
    for (int c = 0; c < m_cols; c++)
    {
        m_cells.at(m_arrayR0 * m_cols + c) = m_defaultValue;
    }

    m_mapR0++;
    m_arrayR0++;
    if (m_arrayR0 == m_rows)
    {
        m_arrayR0 = 0;
    }
}

template<typename T>
void Grid2D<T>::addRowSouth(void)
{
    m_mapR0--;
    if (m_arrayR0 != 0)
    {
        m_arrayR0--;
    }
    else
    {
        m_arrayR0 = m_rows - 1;
    }

    for (int c = 0; c < m_cols; c++)
    {
        m_cells.at(m_arrayR0 * m_cols + c) = m_defaultValue;
    }
}

template<typename T>
bool Grid2D<T>::emptyRow(int r) const
{
    assert(r >= 0 && r < m_rows);

    r = (r + m_arrayR0) % m_rows;

    for (int col = 0; col < m_cols; ++col)
    {
        int c = (col + m_arrayC0) % m_cols;

        if (m_cells.at(r * m_cols + c) != m_defaultValue)
        {
            return false;
        }
    }

    return true;
}

template<typename T>
bool Grid2D<T>::emptyColumn(int c) const
{
    assert(c >= 0 && c < m_cols);

    c = (c + m_arrayC0) % m_cols;

    for (int row = 0; row < m_rows; ++row)
    {
        int r = (row + m_arrayR0) % m_rows;

        if (m_cells.at(r * m_cols + c) != m_defaultValue)
        {
            return false;
        }
    }

    return true;
}



#endif
