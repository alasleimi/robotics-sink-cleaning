#!/usr/bin/env python3
#
# Copyright (c) 2026 Maximilian Mühlbauer
#
# This file is part of pglfd.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Create source files for python bindings."""

import itertools
import sys

manifolds = ["R1*R2", "R1*R3*SO3", "R1*SO2*R2*SO3", "R1*S2*R1*SO3", "R1*R6"]
manifolds_gaussian_process = [
    ("R1", "R2"),
    ("R1", "S2"),
    ("R1", "R3"),
    ("R1", "R3*SO3"),
    ("R1", "SO2*R2*SO3"),
    ("R1", "S2*R1*SO3"),
    ("R1", "R6"),
    ("R3", "R3"),
]
manifolds_tp = [("R1", "R3*SO3"), ("R1", "SO2*R2*SO3"), ("R1", "S2*R1*SO3")]


def _get_manifold(desc: str) -> str:
    if "R" in desc:
        return f"EuclideanManifold<double, {desc[1:]}>"
    if "SO2" in desc:
        return "SO2Manifold<double>"
    if "S2" in desc:
        return "S2Manifold<double>"
    if "SO3" in desc:
        return "SO3Manifold<double>"
    msg = f"Unknown manifold {desc}"
    raise ValueError(msg)


def _get_productmanifold(desc: tuple | list) -> str:
    return "ProductManifold<double, " + ", ".join(_get_manifold(man) for man in desc) + ">"


existing_kmeans = []


def _add_kmeans(desc: tuple) -> str:
    # First ensure that we don't add this twice
    if desc in existing_kmeans:
        return ""
    existing_kmeans.append(desc)

    # Basic Manifold definition
    man = _get_productmanifold(desc)
    return f'\n\n    declare_kmeans<{man}>(m, "KMeans{"".join(desc)}");'


existing_gaussians = []


def _add_gaussian(desc: tuple) -> str:
    # First ensure that we don't add this twice
    if desc in existing_gaussians:
        return ""
    existing_gaussians.append(desc)

    # Basic Manifold definition
    man = _get_productmanifold(desc)
    ret = f'\n\n    declare_gaussian<{man}>(m, "Gaussian{"".join(desc)}")'
    desc_num = list(range(len(desc)))

    # Add margin & conditioning
    for i in range(1, len(desc) + 1):
        for subset in itertools.combinations(desc_num, i):
            name = "".join([str(idx) for idx in subset])
            seq = ", ".join([str(idx) for idx in subset])
            # margin
            ret += f"""
        .def("margin{name}",
            [](std::shared_ptr<Gaussian<{man}>> self) {{
                return self->margin(manifold_selector<{seq}>{{}});
         }})"""

            # get remainder
            if i < len(desc):
                for j in range(1, len(desc) + 1 - i):
                    desc2 = desc_num.copy()
                    for item in subset:
                        desc2.remove(item)
                    for subset2 in itertools.combinations(desc2, j):
                        name2 = "".join([str(idx) for idx in subset2])
                        seq2 = ", ".join([str(idx) for idx in subset2])
                        # condition
                        ret += f"""
        .def("condition{name}_{name2}",
            [](std::shared_ptr<Gaussian<{man}>> self) {{
                return self->condition(manifold_selector<{seq}>{{}}, manifold_selector<{seq2}>{{}});
         }})"""

    return ret + ";"


existing_conditioned_gaussians = []


def _add_conditioned_gaussian(inp: tuple, outp: tuple) -> str:
    # First ensure that we don't add this twice
    if inp + outp in existing_conditioned_gaussians:
        return ""
    existing_conditioned_gaussians.append(inp + outp)

    man_in = _get_productmanifold(inp)
    man_out = _get_productmanifold(outp)
    return (
        f'\n\n    declare_conditioned_gaussian<{man_in}, {man_out}>(m, "ConditionedGaussian{"".join(inp)}_{"".join(outp)}");'
    )


existing_gaussian_regressors = []
existing_gaussian_regressors_r3so3 = []


def _add_gaussian_regressor(inp: list, outp: list) -> str:
    # First ensure that we don't add this twice
    if inp + outp in existing_gaussian_regressors:
        return ""
    existing_gaussian_regressors.append(inp + outp)

    man_in = _get_productmanifold(inp)
    man_out = _get_productmanifold(outp)
    r3so3 = ""
    if inp not in existing_gaussian_regressors_r3so3:
        r3so3 = f'\n\n    declare_gaussian_regressor_r3_so3<{man_in}, double>(m, "GaussianRegressor_R3SO3{"".join(inp)}");'
        existing_gaussian_regressors_r3so3.append(inp)
    return (
        r3so3
        + f'\n\n    declare_gaussian_regressor<{man_in}, {man_out}>(m, "GaussianRegressor{"".join(inp)}_{"".join(outp)}");'
    )


existing_gmms = []


def _add_gmm(desc: tuple) -> str:
    # First ensure that we don't add this twice
    if desc in existing_gmms:
        return ""
    existing_gmms.append(desc)

    # Basic Manifold definition
    man = _get_productmanifold(desc)
    ret = f'    declare_gmm<{man}>(m, "GMM{"".join(desc)}")'
    desc_num = list(range(len(desc)))

    # Add margin & conditioning
    for i in range(1, len(desc) + 1):
        for subset in itertools.combinations(desc_num, i):
            name = "".join([str(idx) for idx in subset])
            seq = ", ".join([str(idx) for idx in subset])
            # margin
            ret += f"""
        .def("margin{name}",
            [](const GMM<{man}> &self) {{
                return self.margin(manifold_selector<{seq}>{{}});
         }})"""

            # get remainder
            if i < len(desc):
                for j in range(1, len(desc) + 1 - i):
                    desc2 = desc_num.copy()
                    for item in subset:
                        desc2.remove(item)
                    for subset2 in itertools.combinations(desc2, j):
                        name2 = "".join([str(idx) for idx in subset2])
                        seq2 = ", ".join([str(idx) for idx in subset2])
                        # condition
                        ret += f"""
        .def("gmr{name}_{name2}",
            [](const GMM<{man}> &self) {{
                return self.gmr(manifold_selector<{seq}>{{}}, manifold_selector<{seq2}>{{}});
         }})"""

    return ret + ";"


existing_gmrs = []


def _add_gmr(inp: tuple, outp: tuple) -> str:
    # First ensure that we don't add this twice
    if inp + outp in existing_gmrs:
        return ""
    existing_gmrs.append(inp + outp)

    man_in = _get_productmanifold(inp)
    man_out = _get_productmanifold(outp)
    return f'\n\n    declare_gmr<{man_in}, {man_out}>(m, "GMR{"".join(inp)}_{"".join(outp)}");'


existing_scalar_kernels = []
existing_product_kernels = []


def _add_kernels(inp: list, outp: list) -> str:
    man_in = _get_productmanifold(inp)
    man_out = _get_productmanifold(outp)

    kernel_rbf = ""
    kernel_matern = ""
    if inp not in existing_scalar_kernels:
        kernel_rbf = f'\n\n    declare_rbf_kernel<{man_in}>(m, "RBF{"".join(inp)}");'
        for p in range(3):
            kernel_matern += f'\n\n    declare_matern_kernel<{man_in}, {p}>(m, "MATERNP{p}_{"".join(inp)}");'
        existing_scalar_kernels.append(inp)

    kernel_product = ""
    kernel_rbf_diag = ""
    kernel_matern_diag = ""
    if inp + outp not in existing_product_kernels:
        kernel_product = (
            f'\n\n    declare_kernel_product<{man_in}, {man_out}>(m, "KERNEL_PRODUCT{"".join(inp)}_{"".join(outp)}");'
        )
        kernel_rbf_diag = (
            f'\n\n    declare_rbf_kernel_diagonal<{man_in}, {man_out}>(m, "RBF_DIAG{"".join(inp)}_{"".join(outp)}");'
        )
        kernel_matern_diag = ""
        for p in range(3):
            kernel_matern_diag = (
                f"\n\n    declare_matern_kernel_diagonal<{man_in}, {man_out}, {p}>(m, "
                f'"MATERNP{p}_DIAG{"".join(inp)}_{"".join(outp)}");'
            )
        existing_product_kernels.append(inp + outp)

    return kernel_rbf + kernel_matern + kernel_product + kernel_rbf_diag + kernel_matern_diag


existing_gps = []


def _add_gaussian_process(inp: list, outp: list) -> str:
    # First ensure that we don't add this twice
    if inp + outp in existing_gps:
        return ""
    existing_gps.append(inp + outp)

    man_in = _get_productmanifold(inp)
    man_out = _get_productmanifold(outp)

    return f'\n\n    declare_gaussian_process<{man_in}, {man_out}>(m, "GP{"".join(inp)}_{"".join(outp)}");'


existing_kmps = []


def _add_kmp(inp: list, outp: list) -> str:
    # First ensure that we don't add this twice
    if inp + outp in existing_kmps:
        return ""
    existing_kmps.append(inp + outp)

    man_in = _get_productmanifold(inp)
    man_out = _get_productmanifold(outp)

    return f'\n\n    declare_kmp<{man_in}, {man_out}>(m, "KMP{"".join(inp)}_{"".join(outp)}");'


existing_nws = []


def _add_nadaraya_watson(inp: list, outp: list) -> str:
    # First ensure that we don't add this twice
    if inp + outp in existing_nws:
        return ""
    existing_nws.append(inp + outp)

    man_in = _get_productmanifold(inp)
    man_out = _get_productmanifold(outp)
    return f'\n\n    declare_nadaraya_watson<{man_in}, {man_out}>(m, "NadarayaWatson{"".join(inp)}_{"".join(outp)}");'


existing_lqts = []


def _add_lqt(man_desc: tuple) -> str:
    if man_desc in existing_lqts:
        return ""
    existing_lqts.append(man_desc)

    man = _get_productmanifold(man_desc)

    return f'\n\n    declare_lqt<{man}>(m, "LQT{"".join(man_desc)}");'


existing_tps: list[list] = []
existing_tps_manif = []


def _add_task_parameterized(inp: list, outp: list) -> str:
    # First ensure that we don't add this twice
    if inp + outp in existing_tps:
        return ""

    man_in = _get_productmanifold(inp)
    man_out = _get_productmanifold(outp)

    tp = ""
    if inp not in existing_tps_manif:
        tp = f'\n\n    declare_tp<{man_in}, double>(m, "TaskParameterized{"".join(inp)}");'
        existing_tps_manif.append(inp)
    return tp + f'\n\n    declare_tp_manif<{man_in}, {man_out}>(m, "TaskParameterizedManif{"".join(inp)}_{"".join(outp)}");'


CPP_TEMPLATE = """#include "pglfd/manif/euclidean.h"  // IWYU pragma: keep
#include "pglfd/manif/product.h"    // IWYU pragma: keep
#include "pglfd/manif/s2.h"         // IWYU pragma: keep
#include "pglfd/manif/so2.h"        // IWYU pragma: keep
#include "pglfd/manif/so3.h"        // IWYU pragma: keep
#include "pglfd/model/pybind/{module_name}.h"
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pyerrors.h>
#include <Eigen/Core> // IWYU pragma: keep
#include <memory>
#include <stdexcept>

namespace pglfd {{
PYBIND11_MODULE({module_name}, m) {{
        py::register_local_exception<std::runtime_error>(
                        m, "PglfdRuntimeWarning", PyExc_RuntimeWarning);

{gen_code}
}}
}}  // namespace pglfd
"""

if __name__ == "__main__":
    kmeans_code = ""
    gaussian_code = ""
    for desc in manifolds:
        man = desc.split("*")
        for i in range(1, len(man) + 1):
            for subset in itertools.combinations(man, i):
                kmeans_code += _add_kmeans(subset)
                gaussian_code += _add_gaussian(subset)
    with open(f"{sys.argv[1]}/kmeans.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="kmeans", gen_code=kmeans_code))
    with open(f"{sys.argv[1]}/gaussian.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="gaussian", gen_code=gaussian_code))

    conditioned_gaussian_code = ""
    for desc in manifolds:
        man = desc.split("*")
        for i in range(1, len(man) + 1):
            for subset in itertools.combinations(man, i):
                if i < len(man):
                    for j in range(1, len(man) + 1 - i):
                        desc2 = man.copy()
                        for item in subset:
                            desc2.remove(item)
                        for subset2 in itertools.combinations(desc2, j):
                            conditioned_gaussian_code += _add_conditioned_gaussian(subset, subset2)
    with open(f"{sys.argv[1]}/conditioned_gaussian.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="conditioned_gaussian", gen_code=conditioned_gaussian_code))

    gmm_code = ""
    for desc in manifolds:
        man = desc.split("*")
        for i in range(1, len(man) + 1):
            for subset in itertools.combinations(man, i):
                gmm_code += _add_gmm(subset)
    with open(f"{sys.argv[1]}/gmm.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="gmm", gen_code=gmm_code))

    gaussian_regressor_code = ""
    for desc in manifolds:
        man = desc.split("*")
        for i in range(1, len(man) + 1):
            for subset in itertools.combinations(man, i):
                if i < len(man):
                    for j in range(1, len(man) + 1 - i):
                        desc2 = man.copy()
                        for item in subset:
                            desc2.remove(item)
                        for subset2 in itertools.combinations(desc2, j):
                            gaussian_regressor_code += _add_gaussian_regressor(list(subset), list(subset2))
    for manifold_pair in manifolds_gaussian_process:
        gaussian_regressor_code += _add_gaussian_regressor(manifold_pair[0].split("*"), manifold_pair[1].split("*"))
    with open(f"{sys.argv[1]}/gaussian_regressor.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="gaussian_regressor", gen_code=gaussian_regressor_code))

    gmr_code = ""
    for desc in manifolds:
        man = desc.split("*")
        for i in range(1, len(man) + 1):
            for subset in itertools.combinations(man, i):
                if i < len(man):
                    for j in range(1, len(man) + 1 - i):
                        desc2 = man.copy()
                        for item in subset:
                            desc2.remove(item)
                        for subset2 in itertools.combinations(desc2, j):
                            gmr_code += _add_gmr(subset, subset2)
    with open(f"{sys.argv[1]}/gmr.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="gmr", gen_code=gmr_code))

    kernels_code = ""
    for manifold_pair in manifolds_gaussian_process:
        kernels_code += _add_kernels(manifold_pair[0].split("*"), manifold_pair[1].split("*"))
    with open(f"{sys.argv[1]}/kernel.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="kernel", gen_code=kernels_code))

    gp_code = ""
    for manifold_pair in manifolds_gaussian_process:
        gp_code += _add_gaussian_process(manifold_pair[0].split("*"), manifold_pair[1].split("*"))
    with open(f"{sys.argv[1]}/gp.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="gp", gen_code=gp_code))

    kmp_code = ""
    for manifold_pair in manifolds_gaussian_process:
        kmp_code += _add_kmp(manifold_pair[0].split("*"), manifold_pair[1].split("*"))
    with open(f"{sys.argv[1]}/kmp.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="kmp", gen_code=kmp_code))

    nadaraya_watson_code = ""
    for manifold_pair in manifolds_gaussian_process:
        nadaraya_watson_code += _add_nadaraya_watson(manifold_pair[0].split("*"), manifold_pair[1].split("*"))
    with open(f"{sys.argv[1]}/nw.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="nw", gen_code=nadaraya_watson_code))

    lqt_code = ""
    for desc in manifolds:
        man = desc.split("*")
        for i in range(1, len(man) + 1):
            for subset in itertools.combinations(man, i):
                lqt_code += _add_lqt(subset)
    with open(f"{sys.argv[1]}/lqt.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="lqt", gen_code=lqt_code))

    task_parameterized_code = ""
    for manifold_pair in manifolds_tp:
        task_parameterized_code += _add_task_parameterized(manifold_pair[0].split("*"), manifold_pair[1].split("*"))
    with open(f"{sys.argv[1]}/tp.cpp", "w") as f:
        f.write(CPP_TEMPLATE.format(module_name="tp", gen_code=task_parameterized_code))
