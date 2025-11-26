#!/usr/bin/env python3
"""独立的轨迹查看器脚本"""
import sys
import argparse
from pathlib import Path

from src.trace_viewer import view_trace


def find_latest_pickle(base_dir: Path = Path('output')) -> Path:
    """
    查找output目录下最新的pickle文件
    
    Args:
        base_dir: 搜索的基础目录
    
    Returns:
        最新的pickle文件路径
    """
    if not base_dir.exists():
        raise FileNotFoundError(f"目录不存在: {base_dir}")
    
    # 递归查找所有pickle文件
    pickle_files = list(base_dir.rglob('*.pickle'))
    
    if not pickle_files:
        raise FileNotFoundError(f"在 {base_dir} 目录下未找到任何pickle文件")
    
    # 按修改时间排序，返回最新的
    latest_pickle = max(pickle_files, key=lambda p: p.stat().st_mtime)
    
    return latest_pickle


def main():
    parser = argparse.ArgumentParser(
        description='查看和导出pickle轨迹文件'
    )
    parser.add_argument(
        'pickle_path',
        type=str,
        nargs='?',
        default=None,
        help='pickle文件路径（可选，默认使用output下最新的pickle）'
    )
    parser.add_argument(
        '--max-rows',
        type=int,
        default=50,
        help='CSV最大导出行数（默认50）'
    )
    parser.add_argument(
        '--no-csv',
        action='store_true',
        help='不导出CSV文件，仅显示摘要'
    )
    
    args = parser.parse_args()
    
    # 确定输入文件
    if args.pickle_path is None:
        # 未指定文件，使用最新的pickle
        try:
            pickle_path = find_latest_pickle()
            print(f"自动选择最新的pickle文件: {pickle_path}")
        except FileNotFoundError as e:
            print(f"错误: {e}")
            sys.exit(1)
    else:
        pickle_path = Path(args.pickle_path)
        if not pickle_path.exists():
            print(f"错误: pickle文件不存在: {args.pickle_path}")
            sys.exit(1)
    
    try:
        view_trace(
            str(pickle_path),
            export_csv=not args.no_csv,
            max_rows=args.max_rows
        )
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
