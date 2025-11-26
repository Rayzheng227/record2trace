#!/usr/bin/env python3
"""Record2Trace - Apollo record转trace工具主程序"""
import sys
import argparse
from pathlib import Path

from src.trace_extractor import extract_trace
from src.trace_viewer import view_trace


def find_latest_record(base_dir: Path = Path('raw_record')) -> Path:
    """
    查找raw_record目录下最新的record文件
    
    Args:
        base_dir: 搜索的基础目录
    
    Returns:
        最新的record文件路径
    """
    if not base_dir.exists():
        raise FileNotFoundError(f"目录不存在: {base_dir}")
    
    # 递归查找所有record文件
    record_files = list(base_dir.rglob('*.record.*'))
    
    if not record_files:
        raise FileNotFoundError(f"在 {base_dir} 目录下未找到任何record文件")
    
    # 按修改时间排序，返回最新的
    latest_record = max(record_files, key=lambda p: p.stat().st_mtime)
    
    return latest_record


def main():
    parser = argparse.ArgumentParser(
        description='将Apollo的record文件转换为trace pickle文件'
    )
    parser.add_argument(
        'record_path',
        type=str,
        nargs='?',
        default=None,
        help='record文件路径（可选，默认使用raw_record下最新的record）'
    )
    parser.add_argument(
        'output_path',
        type=str,
        nargs='?',
        default=None,
        help='输出pickle文件路径（可选，默认保存到output/目录）'
    )
    parser.add_argument(
        '--map',
        type=str,
        default=None,
        help='地图名称（可选，默认从路径推断）'
    )
    parser.add_argument(
        '--view',
        action='store_true',
        help='提取后自动展示轨迹并导出CSV'
    )
    parser.add_argument(
        '--max-rows',
        type=int,
        default=50,
        help='CSV最大导出行数（默认50）'
    )
    
    args = parser.parse_args()
    
    # 确定输入文件
    if args.record_path is None:
        # 未指定文件，使用最新的record
        try:
            record_path = find_latest_record()
            print(f"自动选择最新的record文件: {record_path}")
        except FileNotFoundError as e:
            print(f"错误: {e}")
            sys.exit(1)
    else:
        record_path = Path(args.record_path)
        if not record_path.exists():
            print(f"错误: record文件不存在: {args.record_path}")
            sys.exit(1)
    
    # 确定输出路径
    if args.output_path is None:
        # 默认输出到 output/ 目录
        output_dir = Path('output')
        output_dir.mkdir(exist_ok=True)
        
        # 使用record文件名（保留完整名称）
        record_name = record_path.name  # 例如: 20251015164431.record.00000.20251015164431
        output_path = output_dir / f"{record_name}.pickle"
    else:
        output_path = Path(args.output_path)
        # 确保输出目录存在
        output_path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        # 提取轨迹
        print(f"输出文件: {output_path}")
        result = extract_trace(str(record_path), str(output_path), args.map)
        
        print("\n="*50)
        print("提取统计:")
        print(f"  时间戳数量: {len(result['trace'])}")
        print(f"  Agent数量: {len(result['AgentNames'])}")
        print(f"  地图名称: {result['mapName']}")
        print(f"  测试失败: {result['testFailures']}")
        print("="*50)
        
        # 如果指定了--view，自动展示和导出CSV
        if args.view:
            print("\n开始展示轨迹...")
            view_trace(str(output_path), export_csv=True, max_rows=args.max_rows)
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
