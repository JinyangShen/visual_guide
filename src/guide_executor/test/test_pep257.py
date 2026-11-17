# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # 本项目 docstring 统一用中文书写，因此豁免三条与英文标点强相关的规则：
    #   D213  多行 docstring 摘要应从第二行开始（中文摘要习惯写在首行）
    #   D400  首行需以英文句点结尾
    #   D415  首行需以英文句点/问号/叹号结尾
    # 其余 PEP 257 规则仍然生效。
    rc = main(argv=['--add-ignore', 'D213', 'D400', 'D415', '.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
