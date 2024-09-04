// #include "src/Energy/src/inference.h"
#include "inference.h"

static void decodeOutputs(const float* prob, std::vector<BuffObject>& objects,
                            Eigen::Matrix<float,3,3> &transform_matrix, const int img_w, const int img_h)
{
        // std::vector<BuffObject> proposals;
        // std::vector<int> strides = {8, 16, 32};
        // std::vector<GridAndStride> grid_strides;

        // generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
        // generateYoloxProposals(grid_strides, prob, transform_matrix, BBOX_CONF_THRESH, proposals);
        // qsort_descent_inplace(proposals);

        // if (proposals.size() >= TOPK) 
        //     proposals.resize(TOPK);
        // std::vector<int> picked;
        // nms_sorted_bboxes(proposals, picked, NMS_THRESH);
        // int count = picked.size();
        // objects.resize(count);

        // for (int i = 0; i < count; i++)
        // {
        //     objects[i] = proposals[picked[i]];
        // }
}

BuffDetector::BuffDetector()
{

}

BuffDetector::~BuffDetector()
{
}

//TODO:change to your dir
bool BuffDetector::initModel(string path)
{
    // ie.SetConfig({{CONFIG_KEY(CACHE_DIR), "../.cache"}});
    // // ie.SetConfig({{CONFIG_KEY(GPU_THROUGHPUT_STREAMS),"GPU_THROUGHPUT_AUTO"}});
    // ie.SetConfig({{CONFIG_KEY(GPU_THROUGHPUT_STREAMS),"1"}}  );
    // // Step 1. Read a model in OpenVINO Intermediate Representation (.xml and
    // // .bin files) or ONNX (.onnx file) format
    // network = ie.ReadNetwork(path);
    // if (network.getOutputsInfo().size() != 1)
    //     throw std::logic_error("Sample supports topologies with 1 output only");

    // // Step 2. Configure input & output
    // //  Prepare input blobs
    // InputInfo::Ptr input_info = network.getInputsInfo().begin()->second;
    // input_name = network.getInputsInfo().begin()->first;


    // //  Prepare output blobs
    // if (network.getOutputsInfo().empty())
    // {
    //     std::cerr << "Network outputs info is empty" << std::endl;
    //     return EXIT_FAILURE;
    // }
    // DataPtr output_info = network.getOutputsInfo().begin()->second;
    // output_name = network.getOutputsInfo().begin()->first;

    // // output_info->setPrecision(Precision::FP16);
    // // Step 3. Loading a model to the device
    // // executable_network = ie.LoadNetwork(network, "MULTI:GPU");
    // executable_network = ie.LoadNetwork(network, "GPU");
    // // executable_network = ie.LoadNetwork(network, "CPU");

    // // Step 4. Create an infer request
    // infer_request = executable_network.CreateInferRequest();
    // const Blob::Ptr output_blob = infer_request.GetBlob(output_name);
    // moutput = as<MemoryBlob>(output_blob);
    // // Blob::Ptr input = infer_request.GetBlob(input_name);     // just wrap Mat data by Blob::Ptr
    // if (!moutput)
    // {
    //     throw std::logic_error("We expect output to be inherited from MemoryBlob, "
    //                             "but by fact we were not able to cast output to MemoryBlob");
    // }
    // // locked memory holder should be alive all time while access to its buffer
    // // happens
    // return true;
}

