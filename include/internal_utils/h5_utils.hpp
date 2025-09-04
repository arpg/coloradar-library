#ifndef H5_UTILS_HPP
#define H5_UTILS_HPP


namespace coloradar::internal {


template<> struct H5TypeMap<int>            { static H5::PredType type() { return H5::PredType::NATIVE_INT;    } };
template<> struct H5TypeMap<unsigned int>   { static H5::PredType type() { return H5::PredType::NATIVE_UINT;   } };
template<> struct H5TypeMap<int16_t>        { static H5::PredType type() { return H5::PredType::NATIVE_INT16;  } };
template<> struct H5TypeMap<uint16_t>       { static H5::PredType type() { return H5::PredType::NATIVE_UINT16; } };
template<> struct H5TypeMap<int64_t>        { static H5::PredType type() { return H5::PredType::NATIVE_INT64;  } };
template<> struct H5TypeMap<uint64_t>       { static H5::PredType type() { return H5::PredType::NATIVE_UINT64; } };
template<> struct H5TypeMap<float>          { static H5::PredType type() { return H5::PredType::NATIVE_FLOAT;  } };
template<> struct H5TypeMap<double>         { static H5::PredType type() { return H5::PredType::NATIVE_DOUBLE; } };


template<typename T>
void saveVectorToHDF5(const std::string& name, H5::H5File& file, const std::vector<T>& vec) {
    const hsize_t dims[1] = { vec.size() };
    H5::DataSpace space(1, dims);
    H5::DataSet dataset = file.createDataSet(name, H5TypeMap<T>::type(), space);
    if (!vec.empty()) dataset.write(vec.data(), H5TypeMap<T>::type());
    dataset.close();
    space.close();
}


template<coloradar::PclCloudType CloudT>
std::vector<float> flattenLidarCloud(const std::shared_ptr<CloudT>& cloud, bool collapseElevation, bool removeIntensity) {
    size_t numPoints = cloud->size();
    std::vector<float> data;
    if (numPoints == 0) return data;

    size_t numDims = collapseElevation ? 3 : 4;
    if (removeIntensity) numDims--;

    data.resize(numPoints * numDims);
    for (size_t i = 0; i < numPoints; ++i) {
        data[i * numDims + 0] = (*cloud)[i].x;
        data[i * numDims + 1] = (*cloud)[i].y;
        if (collapseElevation) {
            if (!removeIntensity) data[i * numDims + 2] = (*cloud)[i].intensity;
        } else {
            data[i * numDims + 2] = (*cloud)[i].z;
            if (!removeIntensity) data[i * numDims + 3] = (*cloud)[i].intensity;
        }
    }
    return data;
}

template<coloradar::PclCloudType CloudT>
std::vector<float> flattenRadarCloud(const std::shared_ptr<CloudT>& cloud, const int numElevationBins, const bool hasDoppler) {
    size_t numPoints = cloud->size();
    std::vector<float> data;
    if (numPoints == 0) return data;

    bool hasZ = numElevationBins > 0;
    size_t numDims = hasZ ? 5 : 4;
    if (!hasDoppler) numDims -= 1;

    data.resize(numPoints * numDims);
    for (size_t i = 0; i < numPoints; ++i) {
        data[i * numDims + 0] = (*cloud)[i].x;
        data[i * numDims + 1] = (*cloud)[i].y;
        if (hasZ) {
            data[i * numDims + 2] = (*cloud)[i].z;
            data[i * numDims + 3] = (*cloud)[i].intensity;
            if (hasDoppler) data[i * numDims + 4] = (*cloud)[i].doppler;
        } else {
            data[i * numDims + 2] = (*cloud)[i].intensity;
            if (hasDoppler) data[i * numDims + 3] = (*cloud)[i].doppler;
        }
    }
    return data;
}


template <class T> struct H5MemType;
template <> struct H5MemType<double>   { static const H5::PredType& get(){ return H5::PredType::NATIVE_DOUBLE; } };
template <> struct H5MemType<float>    { static const H5::PredType& get(){ return H5::PredType::NATIVE_FLOAT; } };
template <> struct H5MemType<int16_t>  { static const H5::PredType& get(){ return H5::PredType::NATIVE_INT16; } };
template <> struct H5MemType<hsize_t>  { static const H5::PredType& get(){ return H5::PredType::NATIVE_HSIZE; } };
template <> struct H5MemType<size_t>   { static const H5::PredType& get(){ return H5::PredType::NATIVE_HSIZE; } };

template <class T>
inline std::vector<T> readH5Vector1D(const H5::H5File& file, const std::string& name) {
    H5::DataSet ds = file.openDataSet(name);
    H5::DataSpace sp = ds.getSpace();
    if (sp.getSimpleExtentNdims() != 1) {
        throw std::runtime_error(name + ": expected rank-1 dataset");
    }
    hsize_t dim{};
    sp.getSimpleExtentDims(&dim);
    std::vector<T> v(static_cast<size_t>(dim));
    if (dim > 0) {
        ds.read(v.data(), H5MemType<T>::get());
    }
    return v;
}

template <class T>
inline std::vector<T> readH5Matrix2D(const H5::H5File& file, const std::string& name, size_t& rows, size_t& cols) {
    H5::DataSet ds = file.openDataSet(name);
    H5::DataSpace sp = ds.getSpace();
    if (sp.getSimpleExtentNdims() != 2) {
        throw std::runtime_error(name + ": expected rank-2 dataset");
    }
    hsize_t dims[2]{};
    sp.getSimpleExtentDims(dims);
    rows = static_cast<size_t>(dims[0]);
    cols = static_cast<size_t>(dims[1]);
    std::vector<T> buf(rows * cols);
    if (rows * cols) {
        ds.read(buf.data(), H5MemType<T>::get());
    }
    return buf;
}


} // namespace coloradar::internal



#endif