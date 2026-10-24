// Minimal headless Vulkan compute check: runs saxpy.spv on the first GPU and
// compares every element with the same computation on the CPU.
//   glslangValidator -V saxpy.comp -o saxpy.spv
//   cc -O2 -Wall -o vkcompute vkcompute.c -lvulkan
//   ./vkcompute [saxpy.spv] [n]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <vulkan/vulkan.h>

#define CHECK(x) do { VkResult r_ = (x); if (r_ != VK_SUCCESS) { \
	fprintf(stderr, "%s:%d: %s = %d\n", __FILE__, __LINE__, #x, r_); exit(1); } } while (0)

static double now(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}

static uint32_t *read_spv(const char *path, size_t *size)
{
	FILE *f = fopen(path, "rb");
	if (!f) { perror(path); exit(1); }
	fseek(f, 0, SEEK_END);
	*size = ftell(f);
	rewind(f);
	uint32_t *code = malloc(*size);
	if (fread(code, 1, *size, f) != *size) { perror(path); exit(1); }
	fclose(f);
	return code;
}

// Host-visible, host-coherent storage buffer: no staging needed for a check.
static void make_buffer(VkDevice dev, VkPhysicalDevice pd, VkDeviceSize size,
			VkBuffer *buf, VkDeviceMemory *mem)
{
	VkBufferCreateInfo bi = { .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
		.size = size, .usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		.sharingMode = VK_SHARING_MODE_EXCLUSIVE };
	CHECK(vkCreateBuffer(dev, &bi, NULL, buf));
	VkMemoryRequirements req;
	vkGetBufferMemoryRequirements(dev, *buf, &req);
	VkPhysicalDeviceMemoryProperties mp;
	vkGetPhysicalDeviceMemoryProperties(pd, &mp);
	VkMemoryPropertyFlags want = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
				     VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
	uint32_t type = UINT32_MAX;
	for (uint32_t i = 0; i < mp.memoryTypeCount; i++)
		if ((req.memoryTypeBits & (1u << i)) &&
		    (mp.memoryTypes[i].propertyFlags & want) == want) { type = i; break; }
	if (type == UINT32_MAX) { fprintf(stderr, "no host-visible memory type\n"); exit(1); }
	VkMemoryAllocateInfo ai = { .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
		.allocationSize = req.size, .memoryTypeIndex = type };
	CHECK(vkAllocateMemory(dev, &ai, NULL, mem));
	CHECK(vkBindBufferMemory(dev, *buf, *mem, 0));
}

int main(int argc, char **argv)
{
	const char *spv = argc > 1 ? argv[1] : "saxpy.spv";
	uint32_t n = argc > 2 ? (uint32_t)strtoul(argv[2], NULL, 0) : 1u << 20;
	const float a = 2.5f;

	VkApplicationInfo app = { .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
		.pApplicationName = "vkcompute", .apiVersion = VK_API_VERSION_1_1 };
	VkInstanceCreateInfo ici = { .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
		.pApplicationInfo = &app };
	VkInstance inst;
	CHECK(vkCreateInstance(&ici, NULL, &inst));

	uint32_t npd = 1;
	VkPhysicalDevice pd;
	VkResult er = vkEnumeratePhysicalDevices(inst, &npd, &pd);
	if ((er != VK_SUCCESS && er != VK_INCOMPLETE) || npd == 0) { fprintf(stderr, "no GPU\n"); return 1; }
	VkPhysicalDeviceProperties props;
	vkGetPhysicalDeviceProperties(pd, &props);

	uint32_t nq = 0;
	vkGetPhysicalDeviceQueueFamilyProperties(pd, &nq, NULL);
	VkQueueFamilyProperties qf[16];
	if (nq > 16) nq = 16;
	vkGetPhysicalDeviceQueueFamilyProperties(pd, &nq, qf);
	uint32_t qfi = UINT32_MAX;
	for (uint32_t i = 0; i < nq; i++)
		if (qf[i].queueFlags & VK_QUEUE_COMPUTE_BIT) { qfi = i; break; }
	if (qfi == UINT32_MAX) { fprintf(stderr, "no compute queue\n"); return 1; }

	float prio = 1.0f;
	VkDeviceQueueCreateInfo qci = { .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
		.queueFamilyIndex = qfi, .queueCount = 1, .pQueuePriorities = &prio };
	VkDeviceCreateInfo dci = { .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
		.queueCreateInfoCount = 1, .pQueueCreateInfos = &qci };
	VkDevice dev;
	CHECK(vkCreateDevice(pd, &dci, NULL, &dev));
	VkQueue q;
	vkGetDeviceQueue(dev, qfi, 0, &q);

	VkDeviceSize bytes = (VkDeviceSize)n * sizeof(float);
	VkBuffer bx, by;
	VkDeviceMemory mx, my;
	make_buffer(dev, pd, bytes, &bx, &mx);
	make_buffer(dev, pd, bytes, &by, &my);
	float *x, *y, *ref = malloc(bytes);
	CHECK(vkMapMemory(dev, mx, 0, bytes, 0, (void **)&x));
	CHECK(vkMapMemory(dev, my, 0, bytes, 0, (void **)&y));
	srand(1);
	for (uint32_t i = 0; i < n; i++) {
		x[i] = (float)rand() / RAND_MAX - 0.5f;
		y[i] = (float)rand() / RAND_MAX - 0.5f;
	}
	double t0 = now();
	for (uint32_t i = 0; i < n; i++)
		ref[i] = a * x[i] + y[i];
	double t_cpu = now() - t0;

	size_t spv_size;
	uint32_t *code = read_spv(spv, &spv_size);
	VkShaderModuleCreateInfo smi = { .sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
		.codeSize = spv_size, .pCode = code };
	VkShaderModule sm;
	CHECK(vkCreateShaderModule(dev, &smi, NULL, &sm));

	VkDescriptorSetLayoutBinding b[2] = {
		{ 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL },
		{ 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL },
	};
	VkDescriptorSetLayoutCreateInfo dli = { .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
		.bindingCount = 2, .pBindings = b };
	VkDescriptorSetLayout dsl;
	CHECK(vkCreateDescriptorSetLayout(dev, &dli, NULL, &dsl));
	VkPushConstantRange pcr = { VK_SHADER_STAGE_COMPUTE_BIT, 0, 8 };
	VkPipelineLayoutCreateInfo pli = { .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
		.setLayoutCount = 1, .pSetLayouts = &dsl, .pushConstantRangeCount = 1, .pPushConstantRanges = &pcr };
	VkPipelineLayout pl;
	CHECK(vkCreatePipelineLayout(dev, &pli, NULL, &pl));
	VkComputePipelineCreateInfo cpi = { .sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO,
		.stage = { .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
			   .stage = VK_SHADER_STAGE_COMPUTE_BIT, .module = sm, .pName = "main" },
		.layout = pl };
	VkPipeline pipe;
	CHECK(vkCreateComputePipelines(dev, VK_NULL_HANDLE, 1, &cpi, NULL, &pipe));

	VkDescriptorPoolSize ps = { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 2 };
	VkDescriptorPoolCreateInfo dpi = { .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
		.maxSets = 1, .poolSizeCount = 1, .pPoolSizes = &ps };
	VkDescriptorPool dp;
	CHECK(vkCreateDescriptorPool(dev, &dpi, NULL, &dp));
	VkDescriptorSetAllocateInfo dsai = { .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
		.descriptorPool = dp, .descriptorSetCount = 1, .pSetLayouts = &dsl };
	VkDescriptorSet ds;
	CHECK(vkAllocateDescriptorSets(dev, &dsai, &ds));
	VkDescriptorBufferInfo dbi[2] = { { bx, 0, VK_WHOLE_SIZE }, { by, 0, VK_WHOLE_SIZE } };
	VkWriteDescriptorSet w[2];
	for (int i = 0; i < 2; i++)
		w[i] = (VkWriteDescriptorSet){ .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
			.dstSet = ds, .dstBinding = i, .descriptorCount = 1,
			.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, .pBufferInfo = &dbi[i] };
	vkUpdateDescriptorSets(dev, 2, w, 0, NULL);

	VkCommandPoolCreateInfo cpci = { .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
		.queueFamilyIndex = qfi };
	VkCommandPool cp;
	CHECK(vkCreateCommandPool(dev, &cpci, NULL, &cp));
	VkCommandBufferAllocateInfo cbai = { .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		.commandPool = cp, .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY, .commandBufferCount = 1 };
	VkCommandBuffer cb;
	CHECK(vkAllocateCommandBuffers(dev, &cbai, &cb));
	VkCommandBufferBeginInfo cbbi = { .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	CHECK(vkBeginCommandBuffer(cb, &cbbi));
	vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, pipe);
	vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_COMPUTE, pl, 0, 1, &ds, 0, NULL);
	struct { float a; uint32_t n; } pc = { a, n };
	vkCmdPushConstants(cb, pl, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof pc, &pc);
	vkCmdDispatch(cb, (n + 63) / 64, 1, 1);
	VkMemoryBarrier mb = { .sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER,
		.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT, .dstAccessMask = VK_ACCESS_HOST_READ_BIT };
	vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_HOST_BIT,
			     0, 1, &mb, 0, NULL, 0, NULL);
	CHECK(vkEndCommandBuffer(cb));

	VkFenceCreateInfo fci = { .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
	VkFence fence;
	CHECK(vkCreateFence(dev, &fci, NULL, &fence));
	VkSubmitInfo si = { .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
		.commandBufferCount = 1, .pCommandBuffers = &cb };
	t0 = now();
	CHECK(vkQueueSubmit(q, 1, &si, fence));
	CHECK(vkWaitForFences(dev, 1, &fence, VK_TRUE, UINT64_MAX));
	double t_gpu = now() - t0;

	uint32_t bad = 0;
	float maxerr = 0;
	for (uint32_t i = 0; i < n; i++) {
		float e = fabsf(y[i] - ref[i]);
		if (e > maxerr) maxerr = e;
		if (e > 1e-6f * (1 + fabsf(ref[i]))) bad++;
	}
	printf("device: %s (Vulkan %u.%u.%u)\n", props.deviceName,
	       VK_VERSION_MAJOR(props.apiVersion), VK_VERSION_MINOR(props.apiVersion),
	       VK_VERSION_PATCH(props.apiVersion));
	printf("saxpy n=%u: GPU %.3f ms (submit+wait, first run), CPU %.3f ms\n",
	       n, t_gpu * 1e3, t_cpu * 1e3);
	printf("max |gpu-cpu| = %g, mismatches = %u -> %s\n", maxerr, bad, bad ? "FAIL" : "OK");

	vkDestroyFence(dev, fence, NULL);
	vkDestroyCommandPool(dev, cp, NULL);
	vkDestroyDescriptorPool(dev, dp, NULL);
	vkDestroyPipeline(dev, pipe, NULL);
	vkDestroyPipelineLayout(dev, pl, NULL);
	vkDestroyDescriptorSetLayout(dev, dsl, NULL);
	vkDestroyShaderModule(dev, sm, NULL);
	vkUnmapMemory(dev, mx);
	vkUnmapMemory(dev, my);
	vkDestroyBuffer(dev, bx, NULL);
	vkDestroyBuffer(dev, by, NULL);
	vkFreeMemory(dev, mx, NULL);
	vkFreeMemory(dev, my, NULL);
	vkDestroyDevice(dev, NULL);
	vkDestroyInstance(inst, NULL);
	free(ref);
	free(code);
	return bad ? 1 : 0;
}
