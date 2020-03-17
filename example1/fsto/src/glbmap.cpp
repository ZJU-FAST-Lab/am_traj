#include "fsto/glbmap.h"

using namespace std;
using namespace Eigen;

EuclidDistField::EuclidDistField(Vector3i xyz, Vector3d offset, double scale)
    : sizeXYZ(xyz), originVec(offset), linearScale(scale), sqrDistsPtr(nullptr),
      stepX(1), stepY(sizeXYZ(0)), stepZ(size_t(sizeXYZ(1)) * sizeXYZ(0)),
      linearScaleSqr(linearScale * linearScale)
{
    size_t total = size_t(sizeXYZ(0)) * sizeXYZ(1) * sizeXYZ(2);
    double infDBL = INFINITY;
    sqrDistsPtr = new double[total];
    std::fill_n(sqrDistsPtr, total, infDBL);
}

EuclidDistField::~EuclidDistField()
{
    if (sqrDistsPtr != nullptr)
    {
        delete[] sqrDistsPtr;
    }
}

void EuclidDistField::setOccupied(const Eigen::Vector3d &pos)
{
    int tempXi = (pos(0) - originVec(0)) / linearScale;
    int tempYi = (pos(1) - originVec(1)) / linearScale;
    int tempZi = (pos(2) - originVec(2)) / linearScale;
    if (!(tempXi < 0 || tempYi < 0 || tempZi < 0 || tempXi >= sizeXYZ(0) || tempYi >= sizeXYZ(1) || tempZi >= sizeXYZ(2)))
    {
        sqrDistsPtr[tempXi + tempYi * stepY + tempZi * stepZ] = 0.0;
    }
}

double EuclidDistField::queryDistSqr(const Eigen::Vector3d &pos) const
{
    int tempXi = (pos(0) - originVec(0)) / linearScale;
    int tempYi = (pos(1) - originVec(1)) / linearScale;
    int tempZi = (pos(2) - originVec(2)) / linearScale;
    if (tempXi < 0 || tempYi < 0 || tempZi < 0 || tempXi >= sizeXYZ(0) || tempYi >= sizeXYZ(1) || tempZi >= sizeXYZ(2))
    {
        return 0.0;
    }
    else
    {
        return sqrDistsPtr[tempXi + tempYi * stepY + tempZi * stepZ];
    }
}

double EuclidDistField::queryDist(const Eigen::Vector3d &pos) const
{
    return sqrt(queryDistSqr(pos));
}

void EuclidDistField::updateLinearEDF(double *p, size_t step, size_t N, double sqrLinearScale) const
{
    size_t *v = new size_t[N];
    double *z = new double[N + 1];

    v[0] = 0;
    z[0] = -INFINITY;
    z[1] = INFINITY;
    size_t k = 0;
    for (size_t q = 1; q < N; q++)
    {
        k++;
        double s;
        double tempfq, tempfr;
        do
        {
            k--;

            tempfq = p[q * step] / sqrLinearScale;
            tempfr = p[v[k] * step] / sqrLinearScale;

            if (isinf(tempfq))
            {
                s = INFINITY;
            }
            else if (isinf(tempfr))
            {
                s = -INFINITY;
            }
            else
            {
                s = ((tempfq + q * q) - (tempfr + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
            }

        } while (s <= z[k] && k > 0);

        k++;
        v[k] = q;
        z[k] = s;
        z[k + 1] = INFINITY;
    }

    k = 0;
    for (size_t q = 0; q < N; q++)
    {
        while (z[k + 1] < q)
        {
            k++;
        }
        if (q != v[k])
        {
            p[q * step] = (q * 1.0 - v[k]) * (q * 1.0 - v[k]) * sqrLinearScale + p[v[k] * step];
        }
    }

    delete[] z;
    delete[] v;
}

void EuclidDistField::updateCubicEDF(void)
{
    size_t ini;

    for (int y = 0; y < sizeXYZ(1); y++)
    {
        for (int z = 0; z < sizeXYZ(2); z++)
        {
            ini = y * stepY + z * stepZ;
            updateLinearEDF(sqrDistsPtr + ini, stepX, sizeXYZ(0), linearScaleSqr);
        }
    }

    for (int x = 0; x < sizeXYZ(0); x++)
    {
        for (int z = 0; z < sizeXYZ(2); z++)
        {
            ini = x * stepX + z * stepZ;
            updateLinearEDF(sqrDistsPtr + ini, stepY, sizeXYZ(1), linearScaleSqr);
        }
    }

    for (int x = 0; x < sizeXYZ(0); x++)
    {
        for (int y = 0; y < sizeXYZ(1); y++)
        {
            ini = x * stepX + y * stepY;
            updateLinearEDF(sqrDistsPtr + ini, stepZ, sizeXYZ(2), linearScaleSqr);
        }
    }
}

GlobalMap::GlobalMap(const Config &conf)
    : config(conf), edfPtr(nullptr)
{
}

GlobalMap::~GlobalMap()
{
    if (edfPtr != nullptr)
    {
        delete edfPtr;
    }
}

void GlobalMap::initialize(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    Vector3i xyz((config.r3Bound[1] - config.r3Bound[0]) / config.edfResolution,
                 (config.r3Bound[3] - config.r3Bound[2]) / config.edfResolution,
                 (config.r3Bound[5] - config.r3Bound[4]) / config.edfResolution);

    Vector3d offset(config.r3Bound[0], config.r3Bound[2], config.r3Bound[4]);

    edfPtr = new EuclidDistField(xyz, offset, config.edfResolution);

    size_t cur = 0;
    size_t total = msg->data.size() / msg->point_step;
    float *fdata = (float *)(&msg->data[0]);
    Vector3d tempVec;
    for (size_t i = 0; i < total; i++)
    {
        cur = msg->point_step / sizeof(float) * i;

        if (isnan(fdata[cur + 0]) || isinf(fdata[cur + 0]) ||
            isnan(fdata[cur + 1]) || isinf(fdata[cur + 1]) ||
            isnan(fdata[cur + 2]) || isinf(fdata[cur + 2]))
        {
            continue;
        }
        tempVec << config.unitScaleInSI * fdata[cur + 0],
            config.unitScaleInSI * fdata[cur + 1],
            config.unitScaleInSI * fdata[cur + 2];
        edfPtr->setOccupied(tempVec);
    }
    edfPtr->updateCubicEDF();
}

bool GlobalMap::safeQuery(const Vector3d &p, double safeRadius) const
{
    return edfPtr->queryDistSqr(p) > safeRadius * safeRadius;
}
